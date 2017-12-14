
#  Turtlebot_Search_Rescue
[![Build Status](https://travis-ci.org/zzjkf2009/Turtlebot_Search_Rescue.svg?branch=master)](https://travis-ci.org/zzjkf2009/Turtlebot_Search_Rescue)

[![Coverage Status](https://coveralls.io/repos/github/zzjkf2009/Turtlebot_Search_Rescue/badge.svg?branch=master)](https://coveralls.io/github/zzjkf2009/Turtlebot_Search_Rescue?branch=master)

# Vision-based Turtlebot Controller
This is a ROS package that using color based object detecion (OpenCV) to find the desired direction for tutlebots and let them navigate to that object wiht a PID contorller. The vision-based controller is implement in ROS Gazebo simulation environment with turtlebot as base plantform. A gazebo world is created with red, green, blue objects. During the simulation, user can vary the desired color (HSV) and let the turtlebot navigate to a new target. The HSV of some sample colors are listed below. There will be two version of this package, one for ROS indigo, one for ROS kinetic. 

## Author
* Zejiang Zeng

## License
MIT open-source license,[MIT LICENSE](https://github.com/zzjkf2009/Turtlebot_Search_Rescue/LICENSE.md)
The LICENSE for OpenCV is [OpenCV LICENSE](https://opencv.org/license.html), which is under the BSD license.

## SIP Process
Link:[SIP](https://docs.google.com/spreadsheets/d/1JYr0vUaX_IJUcu1v-DaRxEUFKbrd-KMCGINAMZQEPqs/edit?usp=sharing)

## Dependencies
* Ubuntu 14.04
* ROS indigo
* catkin_make
* TurtleBot_Gazebo
* Package Dependencies
...cv_bridge
...geometry_msgs
...image_transport
...roscpp
...sensor_msgs
...rostest
...rosbag

## Build
```
mkdir catkin_ws/src
cd ./catkin_ws/src
git clone --recursive https://github.com/zzjkf2009/Turtlebot_Search_Rescue
cd ..
catkin_make
```

## Run and Demo
```
cd catkin_ws
source devel/setup.bash
roslaunch ROS_opencv_improcess simulation.launch
```

This should bring you the gazebo simulation environment with two turtlebot inside. robot1 was controlled by the vision-based controller and the robot2 was controlled by the keyboard. The gazebo environment is shown below:
![alt text](https://github.com/zzjkf2009/Turtlebot_Search_Rescue/blob/master/result/Gazebo.png "Gazebo world")
The default desired color is RED and the robot1 will start to turn around in place until it finds fired hydrant. When it sees the target, the threshold image will turns to:

![alt text](https://github.com/zzjkf2009/Turtlebot_Search_Rescue/blob/master/result/fire%20hydrant.png "Fire hydrant")

And the position of target in image will be calculated. The PID controller will help the robot to orientate itself to the its desired target by put the object at the middle of the image and navigate to the it.
The desired color can be changed anytime during the sinulation. For example, after the robot1 reached the fire hydrant, we adjust the HSV value through the controlbar:

![alt text](https://github.com/zzjkf2009/Turtlebot_Search_Rescue/blob/master/result/HSV%20toolbar.png "controlbar")

The template HSV for red, blue and green were given in below section. After we change the LowH to 100 and HighH to 150, the robot1 will find the blue object, which is the postbox by repeating the same process until it finds it as:

![alt text](https://github.com/zzjkf2009/Turtlebot_Search_Rescue/blob/master/result/postboxThreshold.png "postbox")

A couple of arguments can be set through the launch file like, changing the gazebo environmetn, robot name, initial position of the turtlebot by:
```
roslaunch ROS_opencv_improcess simulation.launch robot1_init_pose:="-x -5 -y -5 -z 0" world_file:="$(find ROS_opencv_improcess)/worlds/newTest.world"
```
## Test - rostest
Test implemented can be run as follow:
```
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make run_tests
``` 
## Record - rosbag
*simulation.launch* file supports recording topics in Gazebo simulation excluding the images. This can be done by specifying "enable_record" argument. By default, recording is disabled.

To enable rosbag recording:
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch ROS_opencv_improcess simulation.launch enable_record:=true
```
To inspect rosbag recording result:
```
cd ~/.ros/
rosbag info session.bag
```
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
| ----- |:---------|:----------:| -----:|
| Blue  |  100-150 | 60-255     | 0-255 |
| Green |  40-90   | 10-255     | 0-255 |
| Red   |  0-10    | 40-255     | 0-255 |


### Object Location Detection
The position of the object is calculated using moments function by OpenCV. 1st order spatial moments abound x-axis and y-axis and the 0th order central moments of the binary image is calculated. 
**Note** If there are two or more objects are detected, this method is not working anymore. Also if there is no desired color detected, the turtlebot will stuck at the orinial place.

## PID Controller
If you are not familiar with PID controller, please see: https://www.csimn.com/CSI_pages/PIDforDummies.html
Here, I set the error as the difference bwtween the position, mainly, PoseX of the detected object and the center of the image view, which is error = weight/2 -poseX. Unit in pixel. outout=Pterm +Iterm +Dterm, where output is angular velocity in radius. The PID gain are tuned mannually as: Kp = 0.002, Ki = 0.001, kd = 0.001 

## Issue Note
As metioned above, there are some issue should be noticed, **One** is that if the turtlebot didn't find the desired color for some reason (eg. color didn't exist, target is blocked by other objects), the robot will have to turn in place until it find the color. One sulution could be use keyboard controller to help him to move to a new place and try to look for 
the target at the new place. Or use some other strategies to let it move. **Second** The drawback of deleting the collision check function is that the turtlebot will hit the objects, which may hurt the robots. **Thired** TravisCI didn't pas the Unit test for some reason, it does pass on local compiler. 


