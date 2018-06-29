#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

using namespace std;

static ros::Publisher pub_n280_fix;
static ros::Publisher pub_n280_yaw;


void n280FixCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
	sensor_msgs::NavSatFix out_gps = *msg;
	pub_n280_fix.publish(out_gps);
}

void n280YawCallback(const std_msgs::Float64::ConstPtr& n280_angle)
{
	double yaw = 2 * M_PI - n280_angle->data / 180 * M_PI;
    if (yaw >= M_PI) yaw -= 2*M_PI;
    std_msgs::Float64 out_yaw;
    out_yaw.data = yaw;
    pub_n280_yaw.publish(out_yaw);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "n280pose");
    ros::NodeHandle nh;

    pub_n280_fix = nh.advertise<sensor_msgs::NavSatFix>("n280_fix",10);
    pub_n280_yaw = nh.advertise<std_msgs::Float64>("/n280_yaw", 10);
    ros::Subscriber sub_n280_fix = nh.subscribe("n280/fix", 10, n280FixCallback);
    ros::Subscriber sub_n280_yaw = nh.subscribe("/n280/GPS_N280/orientation", 10, n280YawCallback);

    ros::spin();
    return 0;
}
