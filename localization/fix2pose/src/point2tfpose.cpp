#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "tf/transform_broadcaster.h"
#include "../include/map_frame.h"
#include <iostream>

using namespace std;
geometry_msgs::PoseStamped pose;
MapFrame curr_pos;
double yaw;
string frame_id;
ros::Publisher pose_publisher;

void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    if (msg->status.status == -1) return;
    static tf::TransformBroadcaster br_gps;
    tf::Transform transform_gps;
    tf::Quaternion q_gps;

    curr_pos.GPS2MapFrame(*msg);
    pose.header = msg->header;
    pose.header.frame_id = "map";
    pose_publisher.publish(pose);

    transform_gps.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y,pose.pose.position.z));
    q_gps.setRPY(0, 0, yaw);
    transform_gps.setRotation(q_gps);
    br_gps.sendTransform( tf::StampedTransform(transform_gps, msg->header.stamp, "map" ,frame_id));
}

int main(int argc, char **argv)
{
    double lat,lon,height,yaw;
    string topic_pose,topic_gps;

    sensor_msgs::NavSatFix gps_point;
    MapFrame curr_pos;


    ros::init(argc, argv, "fix2pose");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("lat", lat, 31.02);
    pnh.param<double>("lon", lon, 121.04);
    pnh.param<double>("height", height, 0.0);
    pnh.param<double>("yaw", yaw, 0.0);
    pnh.param<string>("frame_id", frame_id, "/map");
    pnh.param<string>("topic_name",topic_pose,"/point");
    pnh.param<string>("topic_gps", topic_gps, "/rtk/gps");

    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(topic_pose,10);
    gps_point.latitude = lat;
    gps_point.longitude = lon;
    gps_point.altitude = height;
    curr_pos.GPS2MapFrame(gps_point);

    pose.pose.position.x = curr_pos.x;
    pose.pose.position.y = curr_pos.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    ros::Subscriber sub_rtk_gps = nh.subscribe(topic_gps, 10, gpsCallback);
    ros::spin();

    return 0;
}
