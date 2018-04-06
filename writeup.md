## Project: Search and Sample Return

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./output/warped_example.jpg
[image2]: ./output/yellowed.jpg
[image3]: ./output/color_path.jpg
[image4]: ./output/color_obs.jpg
[image5]: ./output/arrow.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
`perspect_transform()` function can change perspect view of camera image. It's shown below.  

![alt text][image1]

`yellow_select()` function can select yellow rock in the image.  

![alt text][image2]  

`color_thresh()` function can select path and obscale in the image.  

![alt text][image3]  
![alt text][image4]  

Functions in Coordinate Transformations section in notebook can change image coordinates to map.  

![alt text][image5]  

All codes can be found in notebook.  
#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

`process_image()` function code is in Rover_Project_Test_Notebook.ipynb.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
`def perception_step(Rover):
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
     # 2) Apply perspective transform
    warped, mask = perspect_transform(img, source, destination)
     # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed, threshed_obs = color_thresh(warped, rgb_thresh=(160, 160, 160))
    threshed_obs = np.absolute(np.float32(threshed_obs))*mask
    rock_threshed = yellow_select(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
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
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.nav_dists = dist
    Rover.nav_angles = angles
    Rover.rock_dists = rock_dist
    Rover.rock_angles = rock_angles
    
    return Rover`

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  



![alt text][image3]


