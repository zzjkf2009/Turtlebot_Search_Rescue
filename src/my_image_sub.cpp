/********************************************************************
 *   MIT License
 *
 *   Copyright (c) 2017 Zejiang Zeng
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

/** @file my_image_sub.cpp
 *  @brief The main source file for visionTurtlebot node
 *
 *  @author Zejiang Zeng
 *  @date   12/2017
 */
#include <ros/ros.h>
#include"../include/ROS_opencv_improcess/TargetDectandControl.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "Vision Control Robot");
  ros::NodeHandle n;
  cv::namedWindow("view");
  cv::startWindowThread();
  cv::namedWindow("Threshold");
  cv::startWindowThread();
  ObjectOrientatedControl Imp(n);
  while (ros::ok) {
    Imp.velocityPublisher();
    ros::spinOnce();
  }
  cv::destroyWindow("view");
  cv::destroyWindow("Threshold");
}
