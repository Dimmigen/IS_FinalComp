#include "ros/ros.h"
#include <ros/console.h>

#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetMap.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/flann.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace cv;
using namespace std;

#define X_OFF_PXL 0.05//0.03
#define Y_OFF_PXL -0.45//0.05

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
int range = 10;


#define X_OFF -0.05
#define Y_OFF 0.45
#define Y_PXL 1


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Subscriber map_sub;
Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;
bool map_avaiable = false;

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
	ROS_INFO("Creating map");
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
	tf::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }
}

class faceApproach
{
public: 	
	MoveBaseClient ac;
	ros::NodeHandle n;

	ros::Subscriber faceSub;
	ros::Publisher sender;
	tf::TransformListener listener;
	
	Approach(ros::NodeHandle &nh): ac("move_base", true), n(nh)
	{		
		ROS_INFO("Starting initializer");
		
		faceSub = n.subscribe<geometry_msgs::PoseStamped>("new_ring", 100, &Approach::set_ring_position, this);
		
		while(!ac.waitForServer(ros::Duration(1.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
	}
	~Approach()
	{
	}
	void getFace(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		approach(msg.target_pose.pose.position.x,msg.target_pose.pose.position.y);
	}
	
	int approach(double x, double y)
	{
		Mat src, src_gray, detected_edges;
		int dist_n = 6;
		int dist_d = 15;
		
		ROS_INFO("Clicked point: x: %f, y: %f", x,y);
		tf::Point pt( x+X_OFF, -(y+Y_OFF), 0.0);
		tf::Point transformed = map_transform.inverse()*pt;
		int map_x = round(transformed.x()/map_resolution);
		int map_y = round(transformed.y()/map_resolution);
		ROS_INFO("Pixels: x: %i, y: %i", map_x, map_y);
		
	
		/// Reduce noise with a kernel 3x3
		blur(cv_map, detected_edges, Size(3,3) );
		/// Canny detector
		ROS_INFO("Did blurring");
		Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
		vector<Point2f> pointsForSearch;
		for(int i = 0; i < detected_edges.rows; ++i)
		{
			for(int j = 0; j < detected_edges.cols; ++j)
			{
				if(detected_edges.at<char>(i,j) != 0)
				pointsForSearch.push_back(Point((float)j,(float)i));
			}
		}
		ROS_INFO("did canny, starting with nearest neighbor");
		flann::KDTreeIndexParams indexParams;
		Mat search = Mat(pointsForSearch).reshape(1,pointsForSearch.size());
		flann::Index kdtree(search, indexParams);
		vector<float> query;
		query.push_back(map_x);
		query.push_back(map_y);
		vector<int> indices;
		vector<float> dists;
	
		int nn = 3;
		kdtree.knnSearch(query, indices, dists, nn);
		cout << query.at(0) << " " << query.at(1) << endl;
		for(unsigned int i = 0; i < indices.size(); ++i)
		{
			cout << indices.at(i)  << " " << dists.at(i) << " " << pointsForSearch.at(indices.at(i)) <<endl;
		}
		
		vector<Point2f> contours;
		for(unsigned int i = 0; i < indices.size(); ++i)
		{
			contours.push_back(pointsForSearch.at(indices.at(i)));
		}
		
		Vec4f line;
		cv::fitLine(contours,line,CV_DIST_L2,0,0.01,0.01);
		tf::Vector3 direction(line[0],line[1],0);
		
		tf::Matrix3x3 rotation;
		rotation.setEulerYPR(-M_PI/2,0,0);
		tf::Vector3 normal = direction*rotation;
		
		ROS_INFO("norm dx: %f, dy: %f", normal.x(), normal.y());
		ROS_INFO("direction dx: %f, dy: %f", direction.x(), direction.y());
		
		int check_x = pointsForSearch.at(indices.at(0)).x+2*normal.x();
		int check_y = pointsForSearch.at(indices.at(0)).y+2*normal.y();
		int v = (int)cv_map.at<unsigned char>(check_y , check_x );
		check_x = pointsForSearch.at(indices.at(0)).x-2*normal.x();
		check_y = pointsForSearch.at(indices.at(0)).y-2*normal.y();
		int w = (int)cv_map.at<unsigned char>( check_y, check_x);
		
		ROS_INFO("v: %d, w: %d", v,w);
		if(v == 0 && w == 255)
		{
			normal *= -1.0;
		}
		else if(v == 255 && w == 0)
		{
			direction *= -1.0;
		}
		else
		{
			ROS_INFO("Something went with the directions wrong!");
			return 1;
		}
		map_x = pointsForSearch.at(indices.at(0)).x;
		map_y = pointsForSearch.at(indices.at(0)).y;
		
		int init_x = round(map_x + dist_n*normal.x());
		int init_y = round(map_y + dist_n*normal.y());
		
		double yaw = atan2(-normal.y(),normal.x());
		pxl_goal(init_x, init_y, yaw);
		
		return 0;
	}
		
	void pxl_goal(int init_x, int init_y, double yaw)
	{
		ROS_INFO("Got pxl: x: %d, y: %d", init_x, init_y);
		tf::Point pt_goal((float)init_x * map_resolution, (float)init_y * map_resolution, 0.0);
		tf::Point init_goal = map_transform * pt_goal;
		
		move_base_msgs::MoveBaseGoal orientation;
		orientation.target_pose.header.frame_id = "map";
		orientation.target_pose.header.stamp = ros::Time::now();
		orientation.target_pose.pose.position.x = init_goal.x()-X_OFF_PXL;
		orientation.target_pose.pose.position.y = -(init_goal.y()-Y_OFF_PXL);
		orientation.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		send_goal(orientation);
	}
	
	void send_goal(const move_base_msgs::MoveBaseGoal& goal)
	{
		ROS_INFO("Got goal");
		ROS_INFO("x: %f, y: %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		ac.sendGoal(goal);
		ROS_INFO("Waiting for result");
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Reached initial pose of approache");
		else
			ROS_INFO("The base failed to reach the initial pose for the approach");
	}
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "faceApproach");
    ros::NodeHandle n;
    faceApproach a(n);

    map_sub = n.subscribe("map", 10, &mapCallback);
    
	ROS_INFO("Start spinning");
	
	while(ros::ok()) 
    {
        ros::spinOnce();
    }
    return 0;

}
