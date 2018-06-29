#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gl8_msgs/Heading.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "tf/transform_broadcaster.h"
#include <iostream>
#include "gl8_utils/map_frame.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
static ros::Publisher pose_publisher;
static double yaw;
string frame_id;
void update_callback(const sensor_msgs::NavSatFixConstPtr &gps_in, const gl8_msgs::HeadingConstPtr &heading_in);


int main(int argc, char **argv)
{
    string topic_pose, topic_gps, topic_yaw, topic_raw_gps;

    ros::init(argc, argv, "fix2pose");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<string>("topic_pose", topic_pose, "/sn");
    pnh.param<string>("topic_gps", topic_gps, "/sn/fix");
    pnh.param<string>("topic_yaw", topic_yaw, "/sn/heading");
    pnh.param<string>("frame_id", frame_id, "/sn_point");

    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(topic_pose,10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> sub_gps_fix(nh, topic_gps, 1);
    message_filters::Subscriber<gl8_msgs::Heading> sub_gps_heading(nh, topic_yaw, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,  gl8_msgs::Heading> UpdatePolicy;
    message_filters::Synchronizer<UpdatePolicy> update_sync(UpdatePolicy(10),sub_gps_fix,sub_gps_heading);
    update_sync.registerCallback(boost::bind(&update_callback, _1, _2));
    ros::spin();
    return 0;
}
void update_callback(const sensor_msgs::NavSatFixConstPtr &gps_in, const gl8_msgs::HeadingConstPtr &heading_in)
{
    if (((gps_in->latitude>30.0)&&(gps_in->latitude<40.0))==false) return;
    if(heading_in->data<10 && heading_in->data>-10)yaw = heading_in->data ;
    if(yaw >= M_PI) yaw -= 2*M_PI;
    if(yaw < -M_PI) yaw += 2*M_PI;

    gl8::common::util::MapFrame curr_pos;
    curr_pos.GPS2MapFrame(gps_in->longitude, gps_in->latitude);

    geometry_msgs::PoseStamped pose;
    static tf::TransformBroadcaster br_gps;
    tf::Transform transform_gps;
    tf::Quaternion q_gps;

    pose.header = gps_in->header;
    pose.header.stamp = gps_in->header.stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = curr_pos.x_;
    pose.pose.position.y = curr_pos.y_;
    pose.pose.position.z = 0;

    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_publisher.publish(pose);

    transform_gps.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y,pose.pose.position.z));
    q_gps.setRPY(0, 0, yaw);
    transform_gps.setRotation(q_gps);
    br_gps.sendTransform( tf::StampedTransform(transform_gps, gps_in->header.stamp, "map" ,frame_id));
}

