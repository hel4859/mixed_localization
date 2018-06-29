#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

string pose_type,path_type;
nav_msgs::Path traj;

ros::Publisher pub_path;

void gpsPoseCallback(geometry_msgs::PoseStamped pose_in)
{
	traj.header.frame_id = "map";
	traj.poses.push_back(pose_in);
	pub_path.publish(traj);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose2path");
	
////	ros::NodeHandle pnh("~");
////	pnh.param<std::string>("psoe_type", pose_type, "truth_pose");
////	pnh.param<std::string>("path_type", path_type, "truth_path");
////	
	
	ros::NodeHandle nh;

//	cout<<"pose_type: "<<pose_type<<endl;
//	cout<<"path_type: "<<path_type<<endl;
	
	ros::Subscriber sub_gps_pose = nh.subscribe("truth_pose", 2, gpsPoseCallback);
	pub_path = nh.advertise<nav_msgs::Path>("truth_path", 2);
	
	ros::spin();
	return 0;
}

