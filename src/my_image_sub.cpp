#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include"../include/ROS_opencv_improcess/TargetDectandControl.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Vision Control Robot");
  ros::NodeHandle n;
  cv::namedWindow("view");
  cv::startWindowThread();
  cv::namedWindow("Threshold");
  cv::startWindowThread();
  ObjectOrientatedControl Imp(n);
  while(ros::ok) {
  Imp.velocityPublisher();
  ros::spinOnce();
}
  cv::destroyWindow("view");
  cv::destroyWindow("Threshold");
}
