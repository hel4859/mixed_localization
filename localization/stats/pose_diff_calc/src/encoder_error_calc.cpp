#include <cmath>
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/NavSatFix.h"
#include "gl8_msgs/VehicleSpeedFeedBack.h"
#include "../include/map_frame.h"
#include "ros/time.h"
using namespace std;
double stats;
double mean;
double rms;
long int N;
bool first_time = true;
bool first_time_chu = true;
bool gpsReceiveFlag = false;
bool encoderReceiveFlag = false;
bool gpsReadFlag = false;
bool gpsInit = false;
bool getGpsStatus();
sensor_msgs::NavSatFix last_gps,first_gps;
double gps_distance = 0;
double encoder_speed;
double encoder_distance = 0;
double sum_gps_distance = 0;
double sum_encoder_distance = 0;
ros::Time time_last,time_last_gps;
ros::Time time_now;
ros::Time time_now_gps;
const double ODOMETRY_FACTOR_NEW = 0.0447;

void rtkGpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_in);
void encoderCallback(const gl8_msgs::VehicleSpeedFeedBack::ConstPtr &data_in);

int main(int argc, char **argv){
    ros::init(argc, argv, "encoder_error_calc");
    ros::NodeHandle nh;
    ros::Subscriber sub_rtk_gps, sub_encoder;
    sub_rtk_gps = nh.subscribe("/strong/fix", 10,rtkGpsCallback);
    sub_encoder = nh.subscribe("/vehicle/speed_feedback",10,encoderCallback);
    ros::spin();
    return 0;
}

bool getGpsStatus(){
    bool flag = gpsReadFlag;
    gpsReadFlag = true;
    return flag;
}


void rtkGpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_in){
    time_last_gps = time_now_gps;
    time_now_gps = ros::Time::now();
    if (!encoderReceiveFlag) {
        ROS_INFO("[Warning] pulse doesn't run!");
        return;
    }
    if (!gpsInit) {
        last_gps = *gps_in;
        first_gps = *gps_in;
        gps_distance = 0.0;
        gpsInit = true;
        N = 0;
        stats = 0;
        mean = 0;
        rms = 0;
    }
    else{
        MapFrame point1,point2;
        point1.GPS2MapFrame(*gps_in);
        point2.GPS2MapFrame(last_gps);
        gps_distance = point1.calcDistance(point2);
        double gps_speed = gps_distance/0.1;
//        point2.GPS2MapFrame(first_gps);
        encoder_distance += (time_now_gps.toSec()-time_now.toSec())*encoder_speed;
        last_gps = *gps_in;
//        if (gps_distance>0.5 or gps_distance<0.2) return;
        if (!first_time_chu) {
            sum_gps_distance += gps_distance; //point1.calcDistance(point2);
            sum_encoder_distance += encoder_distance;
        }
        else{
            first_time_chu = false;
        }
        //if ( encoder_distance/gps_distance<0.8 or encoder_distance/gps_distance>1.2) {
        //    encoder_distance = 0;
        //    return;
        //}
        stats += (encoder_distance/gps_distance-1)*(encoder_distance/gps_distance-1);
        N++;
        mean = sqrt(stats/N);
        ROS_INFO("mean:%f,speed rate:%f",mean,encoder_speed/gps_speed);
        //if (abs(encoder_distance/gps_distance-1)>0.1)
        ROS_INFO("ratio in a step: %f/%f=%f",encoder_distance,gps_distance,encoder_distance/gps_distance);
        ROS_INFO("ratio in total: %f/%f=%f",sum_encoder_distance,sum_gps_distance,sum_encoder_distance/sum_gps_distance);
        encoder_distance = 0;
    }
}

void encoderCallback(const gl8_msgs::VehicleSpeedFeedBack::ConstPtr &data_in){
    double distance;
    encoderReceiveFlag = true;
    time_last = time_now;
    time_now = ros::Time::now();
    if (first_time){first_time=false; return;}
    //distance = 0.5*ODOMETRY_FACTOR*(pulse_in->data[0]+pulse_in->data[1]);
    encoder_speed = 0.995*data_in->rear_wheel_speed;
    distance = encoder_speed*(time_now.toSec()-max(time_last.toSec(),time_now_gps.toSec()));
//    if (distance > 0.2) {
//        distance = ODOMETRY_FACTOR_NEW*min(data_in->rear_left_pulse,data_in->rear_right_pulse);
//        ROS_INFO("Something wrong with pulse");
//    }
    encoder_distance += distance;
}
