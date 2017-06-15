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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/flann.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
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
#define INIT_DIST 120


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
	map_avaiable = true;
}

inline double qdrt(double x){return x*x;}
Point2f comp;
bool distan(Point2f x1, Point2f x2)
{
	double dist1 = sqrt(qdrt(x1.x-comp.x)+qdrt(x1.y-comp.y));
	double dist2 = sqrt(qdrt(x2.x-comp.x)+qdrt(x2.y-comp.y));
	return dist1 < dist2;
}

class Approach
{
public: 	
	MoveBaseClient ac;
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;
	ros::Publisher mapper_pub;
	ros::Subscriber ring_sub;
	ros::Subscriber ringPxl;
	ros::Publisher sender;
	ros::Publisher ringPub;
	tf::TransformListener listener;
	geometry_msgs::PoseStamped ring_pos;
	geometry_msgs::PoseStamped ring_pos2;
	geometry_msgs::PoseStamped ring_avg;
	geometry_msgs::PoseStamped ring_pxl;
	geometry_msgs::PoseStamped init_time;
	geometry_msgs::PoseStamped wait;
	

	bool newRing;
	bool aligning;
	bool getPxl;
	bool newAvg;
	bool pickUp;
	bool check;
	
	int dist;
	int counter;
	
	Approach(ros::NodeHandle &nh): ac("move_base", true), n(nh), dist(INIT_DIST),counter(0),getPxl(false),newRing(false),
		aligning(false), newAvg(false), pickUp(false),check(false)
	{		
		ROS_INFO("Starting initializer");
		
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
		sender = n.advertise<std_msgs::String>("go_on", 10);
		
		ring_sub = n.subscribe<geometry_msgs::PoseStamped>("start_approach", 100, &Approach::set_ring_position, this);
		ringPxl = n.subscribe<geometry_msgs::PoseStamped>("blob_topic", 100, &Approach::set_ring_pxl, this);
		
		ringPub = n.advertise<std_msgs::String>("lets_go",10);
		
		while(!ac.waitForServer(ros::Duration(1.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		
		ring_pos2.pose.position.x = 0;
		ring_pxl.pose.position.x = 0;
		ring_pxl.pose.position.y = 0;
	}
	~Approach()
	{
	}
	void set_ring_pxl(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		if(getPxl and !newAvg)
		{
			ROS_INFO("Got pxl: %d", counter);
			ring_pxl.header.stamp = msg->header.stamp;
			if(counter == 0)
			{
				ring_pxl.pose.position.x = 0;
				ring_pxl.pose.position.y = 0;
			}
			if(counter < 5) 
			{
				ring_pxl.pose.position.x += msg->pose.position.x;
				ring_pxl.pose.position.y += msg->pose.position.y;
				counter++;
			}
			else
			{
				ROS_INFO("Averaging");
				ring_pxl.pose.position.x /= 5;
				ring_pxl.pose.position.y /= 5;
				counter  = 0;
				newAvg = true;
				
			}
		}
	}
		
	void set_ring_position(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		if(!(newRing || aligning))
		{
			ring_pos.header.frame_id = msg->header.frame_id;
			ring_pos.pose = msg->pose;
			ROS_INFO("Got new ring!");
			newRing = true;
		}
		else
		{
			ring_pos2.header.frame_id = msg->header.frame_id;
			ring_pos2.pose = msg->pose;
		}
	}
	
	int set_init(double x, double y)
	{
		Mat src, src_gray, detected_edges;
		int dist_n = 5;
		int dist_d = 10;
		
		ROS_INFO("Clicked point: x: %f, y: %f", x,y);
		tf::Point pt( x+X_OFF, -(y+Y_OFF), 0.0);
		tf::Point transformed = map_transform.inverse()*pt;
		int map_x = round(transformed.x()/map_resolution);
		int map_y = round(transformed.y()/map_resolution);
		ROS_INFO("Pixels: x: %i, y: %i", map_x, map_y);
		
		src = cv_map;
		ROS_INFO("Lets check");
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
		vector<Point2f> contours;
		//vector<Point2f> found;
		for(unsigned int i = 0; i < indices.size(); ++i)
		{
			cout << indices.at(i)  << " " << dists.at(i) << " " << pointsForSearch.at(indices.at(i)) <<endl;
			contours.push_back(pointsForSearch.at(indices.at(i)));
		}
		
		Vec4f line;
		cv::fitLine(contours,line,CV_DIST_L2,0,0.01,0.01);
		tf::Vector3 direction(line[0],line[1],0);
		
		tf::Matrix3x3 rotation;
		rotation.setEulerYPR(-M_PI/2,0,0);
		tf::Vector3 normal = direction*rotation;
		
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
		
		ring_pos.pose.position.x = map_x + dist_n*normal.x();
		ring_pos.pose.position.y = map_y + dist_n*normal.y();
		
		
		int init_x = round(map_x + dist_n*normal.x() - dist_d*direction.x());
		int init_y = round(map_y + dist_n*normal.y() - dist_d*direction.y());
		
		double yaw = atan2(-direction.y(),direction.x());
		//ring_pos.pose.position.z =  yaw;
		
		int check = pxl_goal(init_x, init_y, yaw);
		
		init_time.header.stamp = ros::Time::now();
		wait.header.stamp = ros::Time::now() + ros::Duration(5);
		return check;
	}
	
	int approach_coord(double init_x, double init_y)
	{
		ROS_INFO("Got pxl: x: %f, y: %f", init_x, init_y);
		tf::Point pt_goal((float)init_x * map_resolution, (float)init_y * map_resolution, 0.0);
		tf::Point init_goal = map_transform * pt_goal;
		//tf::Point init_goal(init_x, init_y,0);
		
		ROS_INFO("Entering approach");
		move_base_msgs::MoveBaseGoal pBase, pMap;
		pBase.target_pose.header.frame_id = "base_link";
		pBase.target_pose.pose.position.x = 0.1;
		pBase.target_pose.pose.position.y = -0.1;
		pBase.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		ros::Time current_transform = ros::Time::now();
		ROS_INFO("Waiting for transform");
		listener.waitForTransform(pBase.target_pose.header.frame_id, "map",current_transform, ros::Duration(3.0));
		listener.getLatestCommonTime(pBase.target_pose.header.frame_id, "map", current_transform, NULL);
		pBase.target_pose.header.stamp = current_transform;
		ROS_INFO("Doing the transform");
		listener.transformPose("map", pBase.target_pose, pMap.target_pose);
		
		double dx = init_goal.x()-X_OFF_PXL-pMap.target_pose.pose.position.x;//ring_pose->target_pose.pose.position.x-pMap.target_pose.pose.position.x;
		double dy = (init_goal.y()-Y_OFF_PXL-pMap.target_pose.pose.position.y);//ring_pose->target_pose.pose.position.y-pMap.target_pose.pose.position.y;
		tf::Vector3 v(dx,dy,0);
		v.normalize();
		geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(atan2(-v.y(),v.x()));
		pBase.target_pose.header.frame_id = "base_link";
		pBase.target_pose.pose.position.x = 0.0;
		pBase.target_pose.pose.position.y = 0.0;
		pBase.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		current_transform = ros::Time::now();
		ROS_INFO("Waiting for transform");
		listener.waitForTransform(pBase.target_pose.header.frame_id, "map",current_transform, ros::Duration(3.0));
		listener.getLatestCommonTime(pBase.target_pose.header.frame_id, "map", current_transform, NULL);
		pBase.target_pose.header.stamp = current_transform;
		ROS_INFO("Doing the transform");
		listener.transformPose("map", pBase.target_pose, pMap.target_pose);
		pMap.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(ring_pos.pose.position.z);
		
		ac.sendGoal(pMap);
		ac.waitForResult();
		
		bool go_straight = true;
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Adjusted pose for approache to: x: %f, y: %f", init_goal.x(), init_goal.y());
			return 1;
		}
		else
		{
			ROS_INFO("The base failed to adjust the pose for the approach");
			return 0;
		}
	}

	int approach_pxl(double x, double y)
	{
		ROS_INFO("Got: x: %f y: %f",x,y); 
		if(x < 380){
			turn_left();
			return 1;
		}
		else if(x > 400) {
			turn_right();
			return 1;
		}
		return 0;
	}
		
	int pxl_goal(int init_x, int init_y, double yaw)
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
		return send_goal(orientation);
	}
	
	int send_goal(const move_base_msgs::MoveBaseGoal& goal)
	{
		ROS_INFO("Got goal");
		ROS_INFO("x: %f, y: %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		ac.sendGoal(goal);
		ROS_INFO("Waiting for result");
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Reached initial pose of approache");
			return 0;}
		else{
			ROS_INFO("The base failed to reach the initial pose for the approach");
			return 1;}
	}
	
	void move_straight(int time)
	{
		geometry_msgs::Twist base_cmd;
		base_cmd.linear.x = 0.125;
		ros::Rate r(20);
		ROS_INFO("Moving straight");
		for(int i = 0; i < time; ++i)
		{
			cmd_vel_pub.publish(base_cmd);
			r.sleep(); 
		}
	}
	
	void move_back(int time, double z)
	{
		geometry_msgs::Twist base_cmd;
		base_cmd.linear.x = -0.1;
		base_cmd.angular.z = z;
		ros::Rate r(20);
		ROS_INFO("Moving back");
		for(int i = 0; i < time; ++i)
		{
			cmd_vel_pub.publish(base_cmd);
			r.sleep(); 
		}
	}
	void turn_left()
	{
		int time = 2;
		geometry_msgs::Twist base_cmd;
		base_cmd.linear.x = 0;
		base_cmd.angular.z = 0.1;
		ros::Rate r(20);
		ROS_INFO("Turning left");
		for(int i = 0; i < time; ++i)
		{
			cmd_vel_pub.publish(base_cmd);
			r.sleep(); 
		}
	}
	void turn_right()
	{
		int time = 3;
		geometry_msgs::Twist base_cmd;
		base_cmd.linear.x = 0;
		base_cmd.angular.z = -0.1;
		ros::Rate r(20);
		ROS_INFO("Turning right");
		for(int i = 0; i < time; ++i)
		{
			cmd_vel_pub.publish(base_cmd);
			r.sleep(); 
		}
	}
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "ringApproach");
    ros::NodeHandle n;
    Approach a(n);

    map_sub = n.subscribe("map", 10, &mapCallback);
    
	ROS_INFO("Start spinning");
	int tries = 0;
	int z = 0.1;
	int rings = 0;
	
	while(ros::ok()) 
    {
		if(map_avaiable){
			if(a.newRing) {
				int check = a.set_init(a.ring_pos.pose.position.x, a.ring_pos.pose.position.y);
				a.newRing = false;
				a.aligning = true;
				ROS_INFO("Start aligning now");
				if(check == 1)
				{
					ROS_INFO("Could not align. Not approaching");
					std_msgs::String msg;
					msg.data = "bad";
					ROS_INFO("Asking for new goals");
					//a.sender.publish(msg);
					a.ringPub.publish(msg);
					a.check = false;
					a.getPxl = false;
					a.aligning = false;
				}
			if(a.aligning){
				a.getPxl = true;
				if(a.init_time.header.stamp < a.ring_pxl.header.stamp || a.newAvg)
				{
					a.init_time.header.stamp = ros::Time::now();
					a.wait.header.stamp = ros::Time::now() + ros::Duration(5);
					if(a.newAvg) {
						int check = a.approach_pxl(a.ring_pxl.pose.position.x, a.ring_pxl.pose.position.y);
						if(check == 0) 
						{
							ROS_INFO("Aligning worked. Going straight");
							a.move_straight(a.dist);
							a.aligning = false;
							a.getPxl = false;
							a.pickUp = true;
							tries = 0;
						}
						a.newAvg = false;
					}
				}
				else if(ros::Time::now() > a.wait.header.stamp)
				{
					ROS_INFO("Moving back. i: %i", tries);
					a.getPxl = false;
					a.move_back(5,z);
					a.counter = 0;
					a.getPxl = true;
					z *= -1;
					tries++;
					a.dist += 5;
					
					a.init_time.header.stamp = ros::Time::now();
					a.wait.header.stamp = ros::Time::now() + ros::Duration(5);
				}
				else if(tries > 2)
				{
					ROS_INFO("Could not align. Starting approach_coord");
					int checker = a.approach_coord(a.ring_pos.pose.position.x, a.ring_pos.pose.position.y);
					if(checker == 0) 
					{
						a.move_straight(a.dist);
						a.dist = INIT_DIST;
						a.aligning = false;
						a.getPxl = false;
						a.pickUp = true;
						tries = 0;
					}
					else
					{
						ROS_INFO("Could not do coord approach. Aborting");
						std_msgs::String msg;
						msg.data = "bad";
						ROS_INFO("Asking for new goals");
						//a.sender.publish(msg);
						a.ringPub.publish(msg);
						a.check = false;
						a.getPxl = false;
						a.aligning = false;
					}
				}
			}
			if(a.pickUp)
			{
				ROS_INFO("Picked up ring. Now starting check");
				a.dist = INIT_DIST;
				a.move_back(a.dist,0);
				a.pickUp = false;
				a.check = true;
				a.getPxl = true;
				a.init_time.header.stamp = ros::Time::now();
				a.wait.header.stamp = ros::Time::now() + ros::Duration(5);
			}
			if(a.check)
			{
				a.getPxl = true;
				if(a.init_time.header.stamp < a.ring_pxl.header.stamp || a.newAvg)
				{
					a.aligning = true;
					a.check = false;
					a.getPxl = false;
					ROS_INFO("Redetected the ring. Starting aligning now");
				}
				else if(ros::Time::now() > a.wait.header.stamp)
				{
					ROS_INFO("No ring. Moving back: %d", tries);
					a.getPxl = false;
					a.move_back(5,z);
					a.counter = 0;
					a.getPxl = true;
					z *= -1;
					tries++;
					a.dist += 5;
					
					a.init_time.header.stamp = ros::Time::now();
					a.wait.header.stamp = ros::Time::now() + ros::Duration(5);
				}
				else if(tries > 2)
				{
					tries = 0;
					ROS_INFO("Could not detect ring anymore");
					a.check = false;
					a.getPxl = false;
					/*if(a.ring_pos2.pose.position.x != 0)
					{
						a.ring_pos.pose = a.ring_pos2.pose;
						a.ring_pos2.pose.position.x = 0;
						a.newRing = true;
					}
					else
					{*/
					std_msgs::String msg;
					msg.data = "good";
					ROS_INFO("Asking for new goals");
					//a.sender.publish(msg);
					a.ringPub.publish(msg);
					//}
					
				}
			}
		}
	}	
        ros::spinOnce();
    }
    return 0;

}
