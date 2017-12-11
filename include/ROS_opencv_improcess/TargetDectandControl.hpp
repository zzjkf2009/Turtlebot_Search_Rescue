#include<ros/ros.h>
#include<iostream>
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
int iLowH = 0;
int iHighH = 10;

int iLowS = 40;
int iHighS = 255;

int iLowV = 0;
int iHighV = 100;
int weight;
bool collision ;
float smallest;
public:
  int PoseX ;
  int PoseY ;
  bool find ;
  ObjectOrientatedControl(ros::NodeHandle &n);
  void imageSubscriber();
  void velocityPublisher();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  cv::Mat getThreholdImg( cv::Mat imgOriginal);
  void getPosition(cv::Mat imgThresholded);
  cv::Mat drawPosition(cv::Mat originalImg);
};
