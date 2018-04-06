import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_path = np.zeros_like(img[:,:,0])
    color_obs = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
            
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])        
    # Index the array of zeros with the boolean array and set to 1
    color_path[above_thresh] = 1
    color_obs[below_thresh] = 1
    # Return the binary image
    return color_path, color_obs

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


def rover_coords_obs(binary_img):
    # Identify nonzero pixels
    y_pos, x_pos = np.where(binary_img==0)
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pos += -int(binary_img.shape[1])//2
    y_pos += -int(binary_img.shape[0])
    
    theta = np.pi/2
    x_pixel = x_pos*np.cos(theta)-y_pos*np.sin(theta)
    y_pixel = x_pos*np.sin(theta)+y_pos*np.cos(theta)
    
    y_pixel = -y_pixel
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    outView = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    return warped, outView

def yellow_select(img):
    h_mask = (20, 50)
    s_mask = (100, 255)
    v_mask = (100, 255)
    # 1) Convert to HLS color space
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h, s, v = cv2.split(hsv)
    # 2) Apply a threshold to the S channel
    h_ = np.zeros_like(h, dtype=np.uint8)
    h_[(h > h_mask[0]) & (h < h_mask[1]) & (s>s_mask[0]) & (s<s_mask[1]) & (v>v_mask[0]) & (v<v_mask[1]) ] = 1
    # 3) Return a binary image of threshold result
    binary_output = h_
    return binary_output

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    # 1) Define source and destination points for perspective transform
    dst_size = 8 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])
#     # 2) Apply perspective transform
    warped, mask = perspect_transform(img, source, destination)
#     # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed, threshed_obs = color_thresh(warped, rgb_thresh=(160, 160, 160))
    threshed_obs = np.absolute(np.float32(threshed_obs))*mask
    rock_threshed = yellow_select(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Rover.vision_image[:,:,0] = obstacle 
    # Rover.vision_image[:,:,1] = rock_sample 
    # Rover.vision_image[:,:,2] = navigable 
    Rover.vision_image[:,:,0] = threshed_obs*255 
    Rover.vision_image[:,:,1] = rock_threshed*255 
    Rover.vision_image[:,:,2] = threshed*255 
    
    xpix, ypix = rover_coords(threshed)
    dist, angles = to_polar_coords(xpix, ypix)
    mean_dir = np.mean(angles)
    
    xrock, yrock = rover_coords(rock_threshed)
    rock_dist, rock_angles = to_polar_coords(xrock, yrock)
    
    xobs, yobs = rover_coords(threshed_obs)

    # 5) Convert rover-centric pixel values to world coords
    world_size = 200
    scale = 2*dst_size
    x_pix_world, y_pix_world = pix_to_world(xpix, ypix, 
                 xpos,ypos,yaw, 
                 world_size, scale)
    
    x_obs_world, y_obs_world = pix_to_world(xobs, yobs, 
                 xpos,ypos,yaw,
                 world_size, scale)
    
    x_rock_world, y_rock_world = pix_to_world(xrock, yrock, 
                 xpos,ypos,yaw,
                 world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[y_obs_world, x_obs_world, 0] += 1
    Rover.worldmap[y_rock_world, x_rock_world, 1] += 1
    Rover.worldmap[y_pix_world, x_pix_world, 2] += 1
    #Rover.worldmap[y_obs_world, x_obs_world, 0] = 255
    #Rover.worldmap[y_rock_world, x_rock_world, 1] = 255
    #Rover.worldmap[y_pix_world, x_pix_world, 2] = 255
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = np.mean(dist)
    # Rover.nav_angles = np.mean(angles)
    Rover.nav_dists = dist
    Rover.nav_angles = angles
    Rover.rock_dists = rock_dist
    Rover.rock_angles = rock_angles
   
    return Rover
