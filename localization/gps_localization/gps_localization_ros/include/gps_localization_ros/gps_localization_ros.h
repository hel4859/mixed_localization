//
// Created by wlh on 17-7-13.
//

#ifndef PROJECT_GPS_LOCALIZATION_LIB_H
#define PROJECT_GPS_LOCALIZATION_LIB_H

//ros
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

//std
#include <iostream>




class GPSLocalizationNode
{
public:
    GPSLocalizationNode();

    void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg);

    void yawCallback(const std_msgs::Float64::ConstPtr& msg);

    void yawFilteredCallback(const std_msgs::Float64::ConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_gps_;
    ros::Subscriber sub_yaw_;
    ros::Subscriber sub_yaw_filtered;
    ros::Publisher pub_pose_;
    ros::Publisher pub_path_;

    sensor_msgs::NavSatFix gps_;
    double yaw_;
    geometry_msgs::PoseStamped gps_pose_;
    nav_msgs::Path gps_traj_;

};


#endif //PROJECT_GPS_LOCALIZATION_LIB_H
