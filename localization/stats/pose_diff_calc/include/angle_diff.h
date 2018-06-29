#ifndef ANGLE_DIFF_H
#define ANDLE_DIFF_H

#include <cmath>
#include <string>
#include "std_msgs/Float64.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "sensor.h"
using namespace std;
class SensorInputAngle:virtual public Sensor{
public:
    SensorInputAngle(ros::NodeHandle &node,
                     const string &name,
                     const double &period):Sensor(node,name,period){
        sub_angle = n.subscribe(node_name, 10, &SensorInputAngle::angleCallback,this);
        angle = 0;
        receive_flag = false;
        read_flag = false;
    }
    bool haveReceived(){
        return receive_flag;
    }
    double getAngle(){
        read_flag = true;
        return angle;
    }
    bool haveReaded(){
        return read_flag;
    }

private:
    ros::Subscriber sub_angle;
    bool receive_flag;
    bool read_flag;
    double angle;
    void angleCallback(const std_msgs::Float64::ConstPtr &angle_in){
        angle = angle_in->data;
        receive_flag = true;
        read_flag = false;
    }
};

class SensorOutputAngleDiff:virtual public Sensor{
public:
    SensorOutputAngleDiff(ros::NodeHandle &node,
                          const string &name,
                          const double &period,
                          SensorInputAngle &angle_1,
                          SensorInputAngle &angle_2):
        Sensor(node,name,period),
        sensor_angle_1(angle_1),
        sensor_angle_2(angle_2){
        pub_angle_diff = n.advertise<std_msgs::Float64>(name,10);
        pub_angle_diff1 = n.advertise<std_msgs::Float64>("cexiang",10);
        pub_angle_diff2 = n.advertise<std_msgs::Float64>("yawtrue",10);
        angle_diff.data = 0.0;
        calc_timer = n.createTimer(ros::Duration(T), &SensorOutputAngleDiff::angleDiffCallback,this);
    }
private:
    SensorInputAngle &sensor_angle_1, &sensor_angle_2;
    std_msgs::Float64 angle_diff,angle1,angle2;
    ros::Timer calc_timer;
    ros::Publisher pub_angle_diff,pub_angle_diff1,pub_angle_diff2;
    void angleDiffCallback(const ros::TimerEvent&){
        if (!sensor_angle_1.haveReceived() || !sensor_angle_2.haveReceived()){
            if (!sensor_angle_1.haveReceived())  ROS_INFO("[WARNING] sensor 1 doesn't run.\n");
            if (!sensor_angle_2.haveReceived())  ROS_INFO("[WARNING] sensor 2 doesn't run.\n");
            return;
        }
        if (!sensor_angle_1.haveReaded()||!sensor_angle_2.haveReaded()){
            angle1.data = -sensor_angle_1.getAngle()/180*M_PI;
            angle2.data = sensor_angle_2.getAngle();
            angle2LessThanPI(angle1.data);
            angle2LessThanPI(angle2.data);
            angle_diff.data = angle1.data - angle2.data;
            angle2LessThanPI(angle_diff.data);
            pub_angle_diff.publish(angle_diff);
            pub_angle_diff1.publish(angle1);
            pub_angle_diff2.publish(angle2);
            ROS_INFO("angle difference: %f deg",angle_diff.data * 180 / M_PI);
        }
    }
    void angle2LessThanPI(double &angle){
        if (angle >= M_PI) angle -= 2 * M_PI;
        if (angle < -M_PI) angle += 2 * M_PI;
        return;
    }
};

#endif
