#include <iostream>
#include <stdio.h>
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
  tf::TransformListener listener;
  ros::Publisher blob_pub;
 // image_transport::Publisher image_pub_;

public:
  ImageConverter(ros::NodeHandle n)
    : it_(n), nh_(n)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
   // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);

    blob_pub = n.advertise<geometry_msgs::PoseStamped>("blob_topic",1000);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

  	
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

  // Read image
  Mat src;
  Mat mat_received = cv_ptr->image;
  mat_received.convertTo(src,5);
  Mat src_gray;

  /// Convert it to gray
  cvtColor( mat_received, src_gray, CV_BGR2GRAY );


// Try:
Mat new_image; //= Mat::zeros( mat_received.size(), mat_received.type() );
double alpha = 1.0;
int beta = 10;
src_gray.convertTo(src_gray, -1, alpha, beta);
// -----

  /// Reduce the noise so we avoid false circle detection
  //GaussianBlur( new_image, src_gray, Size(9, 9), 2, 2 );


  vector<Vec3f> circles;

  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 70, 0, 0 );
	

  Mat display = mat_received;
  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      	Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        ROS_INFO("Center - x: %i; y: %i", cvRound(circles[i][0]), cvRound(circles[i][1]));
      	int radius = cvRound(circles[i][2]);
      	// circle center
      	circle(display, center, 3, Scalar(0,255,0), -1, 8, 0 );
      	// circle outline
 	circle(display, center, radius, Scalar(0,0,255), 3, 8, 0 );

	// Transform pixels into coordinates on the map:
	geometry_msgs::PoseStamped pBase, pMap;
	pBase.header.frame_id = msg->header.frame_id;//"camera_rgb_optical_frame";
	//pMap.header.frame_id = "map";
	pBase.pose.position.x = cvRound(circles[i][0]);
	pBase.pose.position.y = cvRound(circles[i][1]);
	pBase.pose.position.z = radius;
	pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	pBase.header.stamp = msg->header.stamp;//ros::Time::now();	
	/*ros::Time current_transform = ros::Time::now();
	ROS_INFO("Waiting for transform");
	listener.waitForTransform(pBase.header.frame_id, "map",current_transform, ros::Duration(3.0));
	listener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform, NULL);
	pBase.header.stamp = current_transform;
	ROS_INFO("Doing the transform");
	listener.transformPose("map", pBase, pMap);*/
	blob_pub.publish(pBase);
  }

  	/// Show your results
  	
	//imshow("Hough Circle Transform Demo",src_gray); // DISPLAY
  	//imshow( "Hough Circle Transform Demo", src );	
	//if(circles.size() > 0)
	ROS_INFO("Detected rings: %i", circles.size());
  }
};

int main(int argc, char** argv)
{
  //namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  //startWindowThread();
  ros::init(argc, argv, "blob2");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ros::Rate r(1);
  ros::spin();
  //destroyWindow("Hough Circle Transform Demo");
  return 0;
}

