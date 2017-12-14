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

/** @file TargetDectandControl.hpp

 *  @brief The  header file for TargetDectandControl class
 *  This class contains a image Subscriber and a velocity Publisher, which publish
 *  desired velocity to turtlebot. The Subscriber will recevive the image from the
 *  camer and Threshold the iamge by HSV to get the position of the desired color.
 *
 * The class contains two main function
 *  @author Zejiang Zeng
 *  @date   12/2017
 */

#ifndef INCLUDE_ROS_OPENCV_IMPROCESS_TARGETDECTANDCONTROL_HPP_
#define INCLUDE_ROS_OPENCV_IMPROCESS_TARGETDECTANDCONTROL_HPP_

#include<ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"

class ObjectOrientatedControl {
 private:
  ros::NodeHandle nh;
  image_transport::Subscriber sub;
  ros::Publisher pub;
  ros::Subscriber ScanSub;
  /**
  * The number defined below are the HSV
  * range for RED color
  */
  int iLowH = 0;
  int iHighH = 10;
  int iLowS = 40;
  int iHighS = 255;
  int iLowV = 0;
  int iHighV = 100;
  int weight;
  bool collision;
  float smallest;

 public:
  int PoseX;
  int PoseY;
  bool find;
  /**
   *   @brief  constructor of the class
   *
   *  Initiate the ObjectOrientatedControl calss with its
   *  subscriber and publisher
   *   @param  ros::NodeHandle
   *   @return double
   */
  explicit ObjectOrientatedControl(ros::NodeHandle &n);
  /**
   *   @brief  start the image subscriber
   *
   *   @param  none
   *   @return none
   */
  void imageSubscriber();
  /**
   *   @brief  start the velocity Publisher
   *
   *   @param  none
   *   @return none
   */
  void velocityPublisher();
  /**
   *   @brief  The callback function for the image subscriber
   *
   * Subscirbe the image from the turtlebot camera and convert the
   * RGB image to HSV image, threshold the image to see if it can find
   * the desired color. When the target is find, the positio will be get
   * based on the getPosition function
   *   @param  snesor_msgs
   *   @return none
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  /**
   *   @brief  threshold the iamge by HSV
   *
   *   @param  cv::Mat
   *   @return cv::Mat
   */
  cv::Mat getThreholdImg(cv::Mat imgOriginal);
  /**
   *   @brief  calculate the position based on the threshold
   * image
   *
   *   @param  cv::Mat
   *   @return none
   */
  void getPosition(const cv::Mat& imgThresholded);
  /**
   *   @brief  draw the detected location of the target in original
   *   image
   *
   *   @param  cv::Mat
   *   @return cv::Mat
   */
  cv::Mat drawPosition(cv::Mat originalImg);
};
#endif  // INCLUDE_ROS_OPENCV_IMPROCESS_TARGETDECTANDCONTROL_HPP_
