//
// Created by wlh on 17-7-13.
//

//custom
#include <gps_localization_ros/gps_localization_ros.h>
#include <gl8_utils/map_frame.h>


GPSLocalizationNode::GPSLocalizationNode()
{
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("gps_pose",10);
    pub_path_ = nh_.advertise<nav_msgs::Path>("gps_path",10);
    sub_gps_ = nh_.subscribe("rtk/fix", 10, &GPSLocalizationNode::gpsCallback,this);
    sub_yaw_ = nh_.subscribe("/n280/GPS_N280/orientation", 10, &GPSLocalizationNode::yawCallback,this);
    sub_yaw_filtered = nh_.subscribe("yaw_filtered", 10, &GPSLocalizationNode::yawFilteredCallback,this);

    ros::spin();
}

void GPSLocalizationNode::gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    gps_ = *msg;

    double east_x,north_y;
    gl8::common::util::MapFrame::GPS2MapFrame(gps_.longitude,gps_.latitude,east_x,north_y);

    gps_pose_.header = msg->header;
    gps_pose_.header.stamp = msg->header.stamp;
    gps_pose_.header.frame_id = "base_link";
    gps_pose_.pose.position.x = east_x;
    gps_pose_.pose.position.y = north_y;
    gps_pose_.pose.position.z = 0.3;
    gps_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);
    pub_pose_.publish(gps_pose_);

    gps_traj_.header.frame_id = "map";
    gps_traj_.poses.push_back(gps_pose_);
    pub_path_.publish(gps_traj_);

    static tf::TransformBroadcaster gps_tf_broadcaster;
    tf::Transform gps_tf;
    tf::Quaternion q;

    gps_tf.setOrigin(tf::Vector3(gps_pose_.pose.position.x, gps_pose_.pose.position.y,gps_pose_.pose.position.z));
    q.setRPY(0, 0, yaw_);
    gps_tf.setRotation(q);
    gps_tf_broadcaster.sendTransform( tf::StampedTransform(gps_tf, ros::Time::now(), "map" ,"base_link"));
}

void GPSLocalizationNode::yawCallback(const std_msgs::Float64::ConstPtr &msg)
{
    double yaw = 2 * M_PI - msg->data / 180 * M_PI;
    if (yaw >= M_PI) yaw -= 2*M_PI;

    yaw_ = yaw;
}

void GPSLocalizationNode::yawFilteredCallback(const std_msgs::Float64::ConstPtr &msg)
{
    yaw_ = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_localization_ros");

    GPSLocalizationNode gps_node;

    return 0;
}
