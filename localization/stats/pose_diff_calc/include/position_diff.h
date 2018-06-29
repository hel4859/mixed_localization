#ifndef POSITION_DIFF_H
#define POSITION_DIFF_H

#include <cmath>
#include <string>
#include <list>
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "global_parameter.h"
#include "map_frame.h"
#include "sensor.h"
using namespace std;

class SensorInputGPS:virtual public Sensor{
public:
    double TIME_DIFF;
    double yaw;
    SensorInputGPS(ros::NodeHandle &node,
                   const string &name,
                   const double &period): Sensor(node,name,period){
        sub_gps = n.subscribe(node_name, 10,&SensorInputGPS::gpsCallback,this);
        receive_flag = 0; // receive_flag is used to help calculate yaw and T
        restart_flag = 0;
        read_flag = 0;
        //casually initiate
        yaw = 0;
        dt = T;
        TIME_DIFF = 0;
    }
    sensor_msgs::NavSatFix readGPS(){
        read_flag = 1;
        return curr_gps;
    }
    bool haveReaded(){
        return read_flag;
    }
    bool haveReceived(){
        return receive_flag;
    }
    bool need2Restart(){
        return restart_flag;
    }
    void doRestart(){
        receive_flag = 0; // receive_flag is used to help calculate yaw and T
        restart_flag = 0;
        read_flag = 0;
        //casually initiate
        yaw = 0;
        dt = T;
        TIME_DIFF = 0;
        list_prev_gps.clear();
    }
private:
    ros::Subscriber sub_gps;
    double dt;
    bool receive_flag;
    bool read_flag;
    bool restart_flag;
    list<sensor_msgs::NavSatFix> list_prev_gps;
    list<sensor_msgs::NavSatFix>::reverse_iterator ir;
    sensor_msgs::NavSatFix curr_gps, prev_gps;
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_in){
        double dist;
        int add_flag = false;
        int list_size = 0;
        TIME_DIFF = ros::Time::now().toSec() - gps_in->header.stamp.toSec();
        if ((std::isnan(gps_in->latitude))||(std::isnan(gps_in->longitude))|| gps_in->status.status==-1) return;
        if (receive_flag){
            MapFrame point1,point2;
            point1.GPS2MapFrame(*gps_in);
            for (ir = list_prev_gps.rbegin();ir!=list_prev_gps.rend();ir++){
                list_size++;
                point2.GPS2MapFrame(*ir);
                dist = point1.calcDistance(point2);
                if (ir == list_prev_gps.rbegin() || dist >= 1){
                    add_flag = true;
                }
                if (dist < 5){
                    continue;
                }
                else{
                    yaw = atan2(point1.y-point2.y,point1.x-point2.x);
                    break;
                }
            }
            if (list_prev_gps.size() > list_size) list_prev_gps.pop_front();
            if (add_flag) list_prev_gps.push_back(*gps_in);
            dt = gps_in->header.stamp.toSec() - curr_gps.header.stamp.toSec();
            if (dt < 0 or dt > 20 * T) restart_flag = 1;
        }
        else{
            receive_flag = 1;
           // SCALE = cos(gps_in->latitude * M_PI/180.0);
            list_prev_gps.push_back(*gps_in);
        }
        read_flag = 0;
        curr_gps = *gps_in;
    }
};

class SensorOutputPositionDiff:virtual public Sensor{
public:
    SensorOutputPositionDiff(ros::NodeHandle &node,
                          const string &name,
                          const double &period,
                          SensorInputGPS &gps_1,
                          SensorInputGPS &gps_2):
        Sensor(node,name,period),
        sensor_gps_1(gps_1),
        sensor_gps_2(gps_2){
        pub_position_diff = n.advertise<std_msgs::Float64>(name,10);
        position_diff.data = 0.0;
        diff_max = 0.0;
        diff_average = 0.0;
        diff_sigma = 0.0;
        tInd = 0;
        calc_timer = n.createTimer(ros::Duration(T), &SensorOutputPositionDiff::positionDiffCallback,this);
    }
private:
    SensorInputGPS &sensor_gps_1;
    SensorInputGPS &sensor_gps_2;
    std_msgs::Float64 position_diff;
    double diff,diff_max,diff_average,diff_sigma;
    int tInd;
    ros::Timer calc_timer;
    ros::Publisher pub_position_diff;
    void positionDiffCallback(const ros::TimerEvent&){
        if (!sensor_gps_1.haveReceived() || !sensor_gps_2.haveReceived()){
            if (!sensor_gps_1.haveReceived())  ROS_INFO("[WARNING] sensor 1 doesn't run.\n");
            if (!sensor_gps_2.haveReceived())  ROS_INFO("[WARNING] sensor 2 doesn't run.\n");
            return;
        }

        if (!sensor_gps_1.haveReaded()||!sensor_gps_2.haveReaded()){
            tInd++;
            MapFrame point1,point2;
            point1.GPS2MapFrame(sensor_gps_1.readGPS());
            point2.GPS2MapFrame(sensor_gps_2.readGPS());
            diff = point1.calcDistance(point2);
            position_diff.data = diff;
            pub_position_diff.publish(position_diff);
            ROS_INFO("position difference: %f m", diff);
            diff_max = max(diff,diff_max);
            diff_average = (diff_average * (tInd-1) + diff) / tInd;
            diff_sigma = (diff_sigma * (tInd-1) + diff * diff) / tInd;
            ROS_INFO("x1-x2= %f m",point1.x-point2.x);
            ROS_INFO("y1-y2= %f m",point1.y-point2.y);
            ROS_INFO("MAX diff is %f",diff_max);
            ROS_INFO("Average: %f",diff_average);
            ROS_INFO("sigma: %f",sqrt(diff_sigma));
        }
    }
};

#endif
