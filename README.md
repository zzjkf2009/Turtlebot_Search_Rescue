
#  Turtlebot_Search_Rescue
[![Build Status](https://travis-ci.org/zzjkf2009/Turtlebot_Search_Rescue.svg?branch=master)](https://travis-ci.org/zzjkf2009/Turtlebot_Search_Rescue)

[![Coverage Status](https://coveralls.io/repos/github/zzjkf2009/Turtlebot_Search_Rescue/badge.svg?branch=master)](https://coveralls.io/github/zzjkf2009/Turtlebot_Search_Rescue?branch=master)

# Vision-based Turtlebot Controller
This the a ROS package that using color based object detecion (OpenCV) to find the desired direction and then navigate to that object wiht a PID contorller.  

## SIP Process
Link:https://docs.google.com/spreadsheets/d/1JYr0vUaX_IJUcu1v-DaRxEUFKbrd-KMCGINAMZQEPqs/edit?usp=sharing

## Color detection and Object tracking

Object detection and segmenration is the most important and challenging fundamental task of computer vision. It is a critical part
in our project for turtlebot to "find" what he need to see. The easiest way to detect and segment an object from an image is the color based methods. The object and the background should jave a significant color difference in order to successfully segment objects using color based method. This is acceptable in simulation in gazebo, but in reality, this assumetion is no longer valid. so there are much more complex and fansy methods, filters and algorithms to classfy and detect different features. In this project, I'm simplely using a color based detector to let the robot find see where he should go. All the image processing are done using OpenCV library. 

### cv_bridge
Link: http://wiki.ros.org/cv_bridge
The cv_bridge package create the interface between ROS and OpenCV by converting ROS images into OpenCV images, and vice versa. It lets us to convert ROS image data type to OpenCV image data type and then process image using OpenCV library. 

### Object detection in HSV Color Space
Usually, we get the image in RGB color space by camera. But HSV color space is the most suitable color space for color based image segmentation. So, I converted the color space of original image of the turtlebot frome RGB to HSV image. HUE is unique for that specific color distribution of the object. But SATURATION and VALUE may be vary according to the lighting condition and environment. More detail about the color spaces can be find at : https://www.learnopencv.com/tag/hsv/
In this project, according to a mannual classification process, I defined the HUE values of a few basic colors as below:
| Color |  HUE     | SATURATION | VALUE |
| ----- |:---------|:----------:|:-----:|
| Blue  |  100-150 | 60-255     | 0-255 |
| Green |  40-90   | 10-255     | 0-255 |
| Red   |  0-10    | 40-255     | 0-255 |

### Object Location Detection
The position of the object is calculated using moments function by OpenCV. 1st order spatial moments abound x-axis and y-axis and the 0th order central moments of the binary image is calculated. 
**Note** If there are two or more objects are detected, this method is not working anymore. 

## PID Controller
If you are not familiar with PID controller, please see: https://www.csimn.com/CSI_pages/PIDforDummies.html
Here, I set the error as the difference bwtween the position, mainly, PoseX of the detected object and the center of the image view, which is error = weight/2 -poseX. Unit in pixel. outout=Pterm +Iterm +Dterm, where output is angular velocity in radius. The PID gain are tuned mannually as: Kp = 0.002, Ki = 0.001, kd = 0.001 

### Obstacle Avoidance 
To be continue

