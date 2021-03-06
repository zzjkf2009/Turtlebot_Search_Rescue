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
/** @file TargetDectest.cpp
 *  @brief Implementation of unit test for ROS TargetDectandControl class
 *
 *  This file contains implementation of unit test for ROS TargetDectandControl class
 *
 *  @author Zejiang Zeng
 *  @date   12/06/2017
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"TargetDectandControl.hpp"
#include"PIDcontroller.hpp"

/**
 *   @brief  A template callback function for imageSubscriber
 *   This empty callback functio is to check the constructor or the TargetDectandControl class
 *   @param  none
 *   @return none
 */

void emptyCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_DEBUG_STREAM("empty image callback");
}

/**
 *   @brief  Verify the imageSubscriber function in TargetDectandControl class
 *
 *   @param  none
 *   @return none
 */

TEST(TargertDectest, testimageSub) {
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::Subscriber sub;
  image_transport::ImageTransport it(nh);
  pub = it.advertise("camera/rgb/image_raw", 100);
  sub = it.subscribe("camera/rgb/image_raw", 1, &emptyCallback);
  EXPECT_EQ(1, pub.getNumSubscribers());
}

/**
 *   @brief  Verify the imageCallback function in ObjectOrientatedControl class
 *   Publish a self-defined RED image (sensor_msgs/image), verify it can detect its
 position. Test pass if it detects the blue object, where PoseX and PoseY is the
 location of the object
 *   @param  none
 *   @return none
 */

TEST(TargertDectest, testimageCallbackNo1) {
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::ImageTransport it(nh);
  pub = it.advertise("camera/rgb/image_raw", 100);
  cv::Mat image(300, 480, CV_8UC3, cv::Scalar(0, 0, 100));
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                 image).toImageMsg();
  ObjectOrientatedControl Imp(nh);
  int count = 0;
  while (count < 5) {
    pub.publish(msg);
    ros::spinOnce();
    ++count;
  }
  EXPECT_NE(0, image.rows);
  EXPECT_NE(-1, Imp.PoseX);
  EXPECT_NE(-1, Imp.PoseY);
  EXPECT_LT(Imp.PoseX, 480);
  EXPECT_LT(Imp.PoseY, 480);
}

/**
 *   @brief  Verify the imageCallback function in ObjectOrientatedControl class
 *   Publish a self-defined BLUE image (sensor_msgs/image), verify it can't detect its
 position. Pass shold pass if it CAN'T find the object in red.
 *   @param  none
 *   @return none
 */

TEST(TargertDectest, testimageCallbackNo2) {
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::ImageTransport it(nh);
  pub = it.advertise("camera/rgb/image_raw", 100);
  cv::Mat image(300, 480, CV_8UC3, cv::Scalar(100, 0, 0));
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                 image).toImageMsg();
  ObjectOrientatedControl Imp(nh);
  int count = 0;
  while (count < 5) {
    pub.publish(msg);
    ros::spinOnce();
    ++count;
  }
  EXPECT_NE(0, image.rows);
  EXPECT_EQ(-1, Imp.PoseX);
  EXPECT_EQ(-1, Imp.PoseY);
  EXPECT_LT(Imp.PoseX, 480);
  EXPECT_LT(Imp.PoseY, 480);
}

/**
 *   @brief  Verify the drawPosition function in ObjectOrientatedControl class
 *
 *   @param  none
 *   @return none
 */

TEST(TargertDectest, testdrawPosition) {
  ros::NodeHandle nh;
  ObjectOrientatedControl Imp(nh);
  cv::Mat image(300, 480, CV_8UC3, cv::Scalar(0, 0, 0));
  Imp.PoseX = 100;
  Imp.PoseY = 150;
  cv::Mat ThresholdImg1 = Imp.drawPosition(image);
  EXPECT_NE(0, image.rows);
  EXPECT_TRUE(Imp.find);
  Imp.PoseX = -1;
  Imp.PoseY = -1;
  cv::Mat ThresholdImg2 = Imp.drawPosition(image);
  EXPECT_FALSE(Imp.find);
  EXPECT_NE(0, image.rows);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TargetDectandControl_tester");
  return RUN_ALL_TESTS();
}
