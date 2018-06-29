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

    sensor_msgs::NavSatFix MapFrame2GPS(){
        double lat = 360.0 / M_PI * atan(exp(y/(SCALE * EARTH_RAD_EQ)))-90.0;
        double lon = 180.0 * x / (SCALE * EARTH_RAD_EQ * M_PI);
        sensor_msgs::NavSatFix gps;
        gps.latitude = lat;
        gps.longitude = lon;
        return gps;
    }

    void GPS2MapFrame(const sensor_msgs::NavSatFix &gps){
        x = SCALE * EARTH_RAD_EQ * gps.longitude * M_PI / 180.0;
        y = SCALE * EARTH_RAD_EQ * log(tan((90.0 + gps.latitude) * (M_PI / 360.0)));
    }
    double deadReckoning(const double  &travel_distance,const double  &yaw_before, const double &turn_radian){
        double yaw = yaw_before + turn_radian/2;
        x += travel_distance * cos(yaw);
        y += travel_distance * sin(yaw);
        yaw += turn_radian/2;
        return yaw;
    }
    double calcDistance(const MapFrame &another){
        return sqrt(pow(x - another.x,2) + pow(y - another.y,2));
    }
};

#endif
