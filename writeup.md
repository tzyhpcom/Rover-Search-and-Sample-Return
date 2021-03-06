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
[image7]: ./output/auto2.png
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
The `perception_step()` and `decision_step()` functions can be found in  `perception.py` and `decision.py` script.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**  
My simulator settings is as follows.  

![alt text][image8]  


Here are the results in autonomous node.  

![alt text][image6]
![alt text][image7]

I wrote codes about picking rocks, but it's easily stuck when the rock is too close to obscale. So I closed it. I think there might be some navigation algorithm for rover to follow the path to rock, like A star. And it will improve the efficiency of seraching.  



