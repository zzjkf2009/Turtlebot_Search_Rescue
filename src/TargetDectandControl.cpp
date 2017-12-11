#include<ros/ros.h>
#include<iostream>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"../include/ROS_opencv_improcess/TargetDectandControl.hpp"
#include"../include/ROS_opencv_improcess/PIDcontroller.hpp"

ObjectOrientatedControl::ObjectOrientatedControl(ros::NodeHandle &n) {
nh = n;
imageSubscriber();
velocityPublisher();
}


void ObjectOrientatedControl::imageSubscriber() {
  image_transport::ImageTransport it(nh);
  sub = it.subscribe("camera/rgb/image_raw", 1, &ObjectOrientatedControl::imageCallback,this);
}


void ObjectOrientatedControl::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat imgThresholded = getThreholdImg(cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::imshow("Threshold", imgThresholded);
    getPosition(imgThresholded);
    cv::Mat DrawedImg = drawPosition(cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::imshow("view", DrawedImg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

cv::Mat ObjectOrientatedControl::getThreholdImg( cv::Mat imgOriginal) {
    weight = imgOriginal.cols;
    cv:: Mat imgHSV;
    cv:: Mat imgThresholded;
    cv::namedWindow("control");
    cvCreateTrackbar("LowH","control",&iLowH,255);
    cvCreateTrackbar("HighH","control",&iHighH,255);
    cvCreateTrackbar("LowS","control",&iLowS,255);
    cvCreateTrackbar("HighS","control",&iHighS,255);
    cvCreateTrackbar("LowV","control",&iLowV,255);
    cvCreateTrackbar("HighV","control",&iHighV,255);
    //Convert the captured frame from BGR to HSV
    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);
    // Threshold the image
    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);
    // morphological opening  ( remove small objects from the foreground)
    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    // morphological closing  ( remove small holes from the foreground)
    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    return imgThresholded;
  }

void ObjectOrientatedControl::getPosition(cv::Mat imgThresholded) {
cv::Moments oMoments = cv::moments(imgThresholded);
double dM01 = oMoments.m01;
double dM10 = oMoments.m10;
double dArea = oMoments.m00;
PoseX = -1;
PoseY = -1;
if (dArea > 10000) {
  // calculate the position of the object
   PoseX = dM10/dArea;
   PoseY = dM01/dArea;
}
}

cv::Mat ObjectOrientatedControl::drawPosition(cv::Mat originalImg) {
  if (PoseX > 0 && PoseY >0) {
     find = true;
      cv::circle(originalImg, cv::Point(PoseX, PoseY), 10,(255,255,255),1,8,0);
  }
  else {
    find = false;
  }
  return originalImg;
}


void ObjectOrientatedControl::velocityPublisher() {
pub = nh.advertise < geometry_msgs::Twist> ("/cmd_vel_mux/input/teleop", 1000);
PIDcontroller pid;
geometry_msgs::Twist input;
//CheckCollision(smallest);
if (find) {
double error = weight/2 - PoseX;
double out = pid.calculatePID(error);
if (abs(error) > weight/4) {
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
  input.angular.z = 0.5;
}
pub.publish(input);
}
