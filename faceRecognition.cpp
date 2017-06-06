#include "/home/team_zeta/armadillo/usr/include/armadillo"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <detection_msgs/Detection.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace arma;

int width = 80;
int height = 80;
double maxDist = 1.0;

string names[6] = {"GirlX","Peter","Ellen","Scarlett","Einstein","Eddison"};

class Recognizer
{
private:
	mat u;
	vec meanV;
	mat w;
	mat classMean;
	vec a;
	cv::Mat image;
	
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber detectionSub;
	
public:
	Recognizer(ros::NodeHandle n): it_(n), nh_(n)
	{
		image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &Recognizer::imageCb, this);
		detectionSub = nh_.subscribe ("dlib_detector/detections", 100, &Recognizer::recognise,this);
		
		string path = "/home/team_zeta/ROS/src/Competition_3/src/matrices/";
		u.load(path + "PCA_U.mat", arma_ascii);
		meanV.load(path + "PCA_mean.mat", arma_ascii);
		w.load(path + "LDA_w.mat", arma_ascii);
		classMean.load(path + "LDA_classMean.mat", arma_ascii);
	}
	~Recognizer() {}
	
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
		image = cv_ptr->image;
	}
	
	void setA(cv::Mat face)
	{
		a.set_size(face.rows*face.cols*3);
		cv::MatIterator_<cv::Vec3b> it, end;
		int k = 0;
		for( it = face.begin<cv::Vec3b>(), end = face.end<cv::Vec3b>(); it != end; ++it)
		{
			a.at(k) = (*it)[0];
            ++k;
            a.at(k) = (*it)[1];
            ++k;
            a.at(k) = (*it)[2];
            ++k;
		}
	}
	
	void recognise(const detection_msgs::Detection& msg)
	{
		cv::Mat src;
		//cv::Mat gray;
		cv::Rect r(msg.x, msg.y, width, height);
		bool is_inside = (r & cv::Rect(0, 0, image.cols, image.rows)) == r;
		if(is_inside)
		{
			src = image(r);
			//cvtColor( src, gray, CV_BGR2GRAY );
			setA(src);
			
			projectPCA();
			projectLDA();
			compare();
		}
	}
	void projectPCA()
	{
	//	cout << a << endl;
		//cout << "Project on PCA" << endl;
		a = u.t()*(a-meanV);
	//	cout << a << endl;
		
	}

	void projectLDA()
	{
		//cout << "Project on LDA" << endl;
		a = w.t()*a;
	//	cout << a << endl;
	}
	int compare()
	{
		vec dist(classMean.n_cols);
		for(unsigned int i = 0; i < classMean.n_cols; ++i)
		{
			vec diff = a - classMean.col(i);
			dist.at(i) = norm(diff);
		}
		int pos = index_min(dist);
		if(dist.at(pos) < maxDist)
		{
			cout << "Person: " << names[pos]  << endl;
			cout << "Distance: " << dist.at(pos) << endl;
		}
		else cout << "Wrong distance: " << dist.at(pos) << endl;
		//cout << dist << endl;
		return pos;
	}
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "faceRecognition");
  ros::NodeHandle n;
  Recognizer recog(n);
  ros::Rate r(1);
  ROS_INFO("Starting to spin");
  ros::spin();
  return 0;
}
