//
// Created by guolindong on 17-10-29.
//

#ifndef PROJECT_LOAM_ODOM_PROC_H
#define PROJECT_LOAM_ODOM_PROC_H


#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"

#include <mrpt/utils/CFileOutputStream.h>


using namespace std;
using namespace mrpt;

void odom_callback(const nav_msgs::Odometry& msg);

mrpt::utils::CFileOutputStream output_file;
ros::Subscriber sub_odom;
ros::Publisher pub_odom;

#endif //PROJECT_LOAM_ODOM_PROC_H
