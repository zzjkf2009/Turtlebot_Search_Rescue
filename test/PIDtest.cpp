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
/** @file PIDtest.cpp
 *  @brief Implementation of unit test for ROS PIDcontroller class
 *
 *  This file contains implementation of unit test for ROS PIDcontroller class
 *
 *  @author Zejiang Zeng
 *  @date   12/06/2017
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/ROS_opencv_improcess/PIDcontroller.hpp"

/**
 *   @brief  Verify the Pterm function in PIDcontroller class
 *
 *   @param  none
 *   @return none
 */

TEST(TestPIDcontroller, testPterm) {
  PIDcontroller pid;
  EXPECT_EQ(0.2, pid.Pterm(100));
  EXPECT_EQ(0.3, pid.Pterm(150));
}

/**
 *   @brief  Verify the Iterm function in PIDcontroller class
 *
 *   @param  none
 *   @return none
 */

TEST(TestPIDcontroller, testIterm) {
  PIDcontroller pid;
  EXPECT_EQ(0.066, pid.Iterm(100));
  EXPECT_EQ(0.165, pid.Iterm(150));
}

/**
 *   @brief  Verify the Dterm function in PIDcontroller class
 *
 *   @param  none
 *   @return none
 */

TEST(TestPIDcontroller, testDterm) {
  PIDcontroller pid;
  EXPECT_EQ(0.0, pid.Dterm(100));
  EXPECT_EQ(0.0, pid.Dterm(150));
}

/**
 *   @brief  Verify the Dterm function in PIDcontroller class
 *
 *   @param  none
 *   @return none
 */

TEST(TestPIDcontroller, testcalculatePID) {
  PIDcontroller pid;
  EXPECT_DOUBLE_EQ(0.266, pid.calculatePID(100));
  EXPECT_DOUBLE_EQ(0.465, pid.calculatePID(150));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PID_tester");
  return RUN_ALL_TESTS();
}

