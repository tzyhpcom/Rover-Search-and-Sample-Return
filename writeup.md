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
[image6]: ./output/autonomous.png
[image7]: ./output/auto2.jpg
[image8]: ./rover_setting.png

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
Here is `perception_step()` and `decision_step()` functions.  
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

`def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward: # and np.abs(np.mean(Rover.nav_angles * 180/np.pi))<40:
                print("+++++++++++++forward "+str(np.mean(np.abs(Rover.nav_angles) * 180/np.pi)))
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # rock
                #if len(Rover.rock_angles)>0: # and np.min(Rover.rock_dists)<100:
                #    print("++++++++++find rock")
                #    if np.min(Rover.rock_dists)<10:
                #        print("++++++++++start pick rock")
                #        Rover.throttle = 0
                #        Rover.brake = Rover.brake_set
                #        Rover.steer = 0
                #    else:
                #        print("++++++++++move to rock")
                #        if Rover.vel > 0.5:
                #            Rover.throttle = -Rover.throttle_set
                #        elif Rover.vel < 0.5:
                #            Rover.throttle = Rover.throttle_set
                #            print("----------haha")
                #        else:
                #           Rover.throttle = 0
                #        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                #else:
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            #elif len(Rover.nav_angles) < Rover.stop_forward:
            else:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            print("+++++++++++++stop")
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward or np.abs(np.mean(Rover.nav_angles * 180/np.pi))>30:
                    print("+++++++++++++stop and rotate")
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                #if len(Rover.nav_angles) >= Rover.go_forward:
                else:
                    print("+++++++++++++stop and forward")
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = 0
        Rover.steer = -15
        Rover.brake = 0
    
    
    

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover`

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**  
My simulator settings is as follows.  

![alt text][image8]  


Here are the results in autonomous node.  

![alt text][image6]
![alt text][image7]

I wrote codes about picking rocks, but it's easily stuck when the rock is too close to obscale. So I closed it. I think there might be some navigation algorithm for rover to follow the path to rock, like A star. And it will improve the efficiency of seraching.  



