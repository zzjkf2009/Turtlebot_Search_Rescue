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
int iLowH = 110;
int iHighH = 160;

int iLowS = 60;
int iHighS = 255;

int iLowV = 0;
int iHighV = 100;
int PoseX ;
int PoseY ;
int weight;
bool find ;
bool collision ;
double smallest;
public:
  ObjectOrientatedControl();
  void imageSubscriber();
  void velocityPublisher();
  void scanSubscriber();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void DisReceive(const sensor_msgs::LaserScan::ConstPtr& scan);
  cv::Mat getThreholdImg( cv::Mat imgOriginal);
  void getPosition(cv::Mat imgThresholded);
  cv::Mat drawPosition(cv::Mat originalImg);
  bool CheckCollision(double dis);
};
