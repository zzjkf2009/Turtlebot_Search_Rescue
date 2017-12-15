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

/** @file TargetDectandControl.cpp

 *  @brief The  source file for ObjectOrientatedControl class
 *  This class contains a image Subscriber and a velocity Publisher, which publish
 *  desired velocity to turtlebot. The Subscriber will recevive the image from the
 *  camer and Threshold the iamge by HSV to get the position of the desired color.

 *  @author Zejiang Zeng
 *  @date   12/2017
 */
#include<ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"TargetDectandControl.hpp"
#include"PIDcontroller.hpp"

ObjectOrientatedControl::ObjectOrientatedControl() {
  imageSubscriber();
  pub = nh.advertise < geometry_msgs::Twist
      > ("/cmd_vel_mux/input/teleop", 1000);
}

void ObjectOrientatedControl::imageSubscriber() {
  image_transport::ImageTransport it(nh);
  sub = it.subscribe("camera/rgb/image_raw", 1,
                     &ObjectOrientatedControl::imageCallback, this);
}

void ObjectOrientatedControl::imageCallback(
    const sensor_msgs::ImageConstPtr& msg) {
  try {
    ROS_INFO_STREAM("get image");
    cv::Mat imgThresholded = getThreholdImg(
        cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::imshow("Threshold", imgThresholded);
    cv::waitKey(1);
    getPosition(imgThresholded);
    cv::Mat DrawedImg = drawPosition(cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::imshow("view", DrawedImg);
    cv::waitKey(1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

cv::Mat ObjectOrientatedControl::getThreholdImg(cv::Mat imgOriginal) {
  weight = imgOriginal.cols;
  cv::Mat imgHSV;
  cv::Mat imgThresholded;
  cv::namedWindow("control");
  cvCreateTrackbar("LowH", "control", &iLowH, 255);
  cvCreateTrackbar("HighH", "control", &iHighH, 255);
  cvCreateTrackbar("LowS", "control", &iLowS, 255);
  cvCreateTrackbar("HighS", "control", &iHighS, 255);
  cvCreateTrackbar("LowV", "control", &iLowV, 255);
  cvCreateTrackbar("HighV", "control", &iHighV, 255);
  // Convert the captured frame from BGR to HSV
  cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);
  // Threshold the image
  cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV),
              cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);
  // morphological opening  ( remove small objects from the foreground)
  cv::erode(imgThresholded, imgThresholded,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(imgThresholded, imgThresholded,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  // morphological closing  ( remove small holes from the foreground)
  cv::dilate(imgThresholded, imgThresholded,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::erode(imgThresholded, imgThresholded,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  return imgThresholded;
}

void ObjectOrientatedControl::getPosition(const cv::Mat& imgThresholded) {
  cv::Moments oMoments = cv::moments(imgThresholded);
  double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;
  PoseX = -1;
  PoseY = -1;
  if (dArea > 10000) {
    // calculate the position of the object
    PoseX = dM10 / dArea;
    PoseY = dM01 / dArea;
  }
}

cv::Mat ObjectOrientatedControl::drawPosition(cv::Mat originalImg) {
  if (PoseX > 0 && PoseY > 0) {
    find = true;
    cv::circle(originalImg, cv::Point(PoseX, PoseY), 10, (255, 255, 255), 1, 8,
               0);
  } else {
    find = false;
  }
  return originalImg;
}

void ObjectOrientatedControl::velocityPublisher() {
  PIDcontroller pid;
  geometry_msgs::Twist input;
  if (find) {
    double error = weight / 2 - PoseX;
    double out = pid.calculatePID(error);
    if (abs(error) > weight / 4) {
      input.linear.x = 0.0;
      input.linear.y = 0;
      input.linear.z = 0;
      input.angular.x = 0;
      input.angular.y = 0;
      input.angular.z = out;
    } else {
      input.linear.x = 1.0;
      input.linear.y = 0;
      input.linear.z = 0;
      input.angular.x = 0;
      input.angular.y = 0;
      input.angular.z = out;
    }
  } else {
    input.linear.x = 0;
    input.linear.y = 0;
    input.linear.z = 0;
    input.angular.x = 0;
    input.angular.y = 0;
    input.angular.z = 0.8;
  }
  pub.publish(input);
}
