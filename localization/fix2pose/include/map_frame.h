#ifndef _MAP_FRAME_H
#define _MAP_FRAME_H
#include "global_parameter.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
class MapFrame{
public:
    double x;//east
    double y;//north

    MapFrame(const double x_in=0.0,
             const double y_in=0.0):
        x(x_in),
        y(y_in){}

    MapFrame(const MapFrame &pos):
        x(pos.x),
        y(pos.y){}

    ~MapFrame(){}

    sensor_msgs::NavSatFix MapFrame2GPS();

    void GPS2MapFrame(const sensor_msgs::NavSatFix &gps);
    double deadReckoning(const double &travel_distance,const double &yaw_before, const double &turn_radian);
    double calcDistance(const MapFrame &another);
};

#endif
