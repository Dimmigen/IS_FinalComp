#include <iostream>
#include <stdio.h>
#include <sstream>
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <detection_msgs/Detection.h>

#ifndef PI
#define PI 3.14159265359
#endif

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber detectionSub;
  Mat mat_received;
  int numb;

public:
  ImageConverter(ros::NodeHandle n)
    : it_(n), nh_(n),numb(0)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
	detectionSub = nh_.subscribe ("dlib_detector/detections", 100, &ImageConverter::savePic,this);
    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

//  	ROS_INFO("Got image");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
//	ROS_INFO("Storing matrix");
	mat_received = cv_ptr->image;
  }
  
  void savePic(const detection_msgs::Detection& msg)
  {
	ROS_INFO("Starting savePic");
	Mat src;
	//mat_received.convertTo(src,3);
	stringstream dummy;
	dummy <<  "/images/pic" << numb << ".png";
	namedWindow("image", WINDOW_AUTOSIZE);
	imshow("image", mat_received);
	waitKey(10);
	imwrite( dummy.str(), mat_received );
	ROS_INFO("Detected face and stored it: %i", numb);
	++numb;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "savePic");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ros::Rate r(1);
  ROS_INFO("Starting to spin");
  ros::spin();
  return 0;
}
