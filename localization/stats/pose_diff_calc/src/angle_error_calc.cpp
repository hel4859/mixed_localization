#include <cmath>
#include "ros/ros.h"
#include "ros/console.h"
#include "gl8_msgs/VehicleIMU.h"
#include "gl8_msgs/Heading.h"

using namespace std;

bool gpsReceiveFlag = false;
bool imuReceiveFlag = false;
bool gpsReadFlag = false;
bool gpsInit = false;
bool getGpsStatus();
double last_gps_angle,first_gps_angle;
double gps_angle_diff = 0;
double imu_angle_diff = 0;
double imu_max = 0;
double sum_gps_angle = 0;
double sum_imu_angle = 0;
double T = 0.01;
int times = 0;
void gpsAngleCallback(const gl8_msgs::Heading::ConstPtr &gps_angle_in);
void imuCallback(const gl8_msgs::VehicleIMU::ConstPtr &imu_in);
double angle2LessThanPI(double angle);
int main(int argc, char **argv){
    ros::init(argc, argv, "angle_error_calc");
    ros::NodeHandle nh;
    ros::Subscriber sub_gps_angle, sub_imu;
    sub_gps_angle = nh.subscribe("/sn/heading", 10,gpsAngleCallback);
    sub_imu = nh.subscribe("/vehicle/imu",10,imuCallback);
    ros::spin();
    return 0;
}
bool getGpsStatus(){
    bool flag = gpsReadFlag;
    gpsReadFlag = true;
    return flag;
}


void gpsAngleCallback(const gl8_msgs::Heading::ConstPtr &gps_angle_in){
    double curr_angle;
    times++;
    if (times % 20!=0) return;
    gpsReceiveFlag = true;
    if (!imuReceiveFlag) {
        ROS_INFO("[Warning] imu doesn't run!");
        return;
    }
    curr_angle = gps_angle_in->data;
    gpsReadFlag = false;

    if (!gpsInit) {
        first_gps_angle = curr_angle;
        last_gps_angle = first_gps_angle;
        gps_angle_diff = 0.0;
        gpsInit = true;
    }
    else{
        gps_angle_diff = curr_angle - last_gps_angle;
        angle2LessThanPI(gps_angle_diff);
        sum_gps_angle = curr_angle - first_gps_angle;
        angle2LessThanPI(sum_gps_angle);
        last_gps_angle = curr_angle;
        ROS_INFO("error in a step: %f-%f=%f",180*imu_angle_diff/M_PI,180*gps_angle_diff/M_PI,180*angle2LessThanPI(imu_angle_diff-gps_angle_diff)/M_PI);
        ROS_INFO("error in total: %f-%f=%f",180*sum_imu_angle/M_PI,180*sum_gps_angle/M_PI,180*angle2LessThanPI(sum_imu_angle-sum_gps_angle)/M_PI/times/T);
    }
}

void imuCallback(const gl8_msgs::VehicleIMU::ConstPtr &imu_in){
    double angle;
    imuReceiveFlag = true;
    if (!gpsReceiveFlag){
        ROS_INFO("[Warning] gps doesn't run!");
        return;
    }
    angle = imu_in->yaw_z  * T;//* M_PI/180.0
    sum_imu_angle += angle;
    sum_imu_angle = angle2LessThanPI(sum_imu_angle);
    if (!getGpsStatus()){
        imu_angle_diff = angle;
    }
    else{
        imu_angle_diff += angle;
    }
    imu_angle_diff = angle2LessThanPI(imu_angle_diff);
    imu_max = max(abs(imu_angle_diff),imu_max);
    //ROS_INFO("max_imu:%f\n",180*imu_max/M_PI);
}

double angle2LessThanPI(double angle){
    if (angle >= M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}