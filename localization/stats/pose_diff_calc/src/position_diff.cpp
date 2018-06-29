//This project contains very few comments.
//More information is detailed in the "description" of "../package.xml".
#include "../include/position_diff.h"
using namespace std;
int main(int argc, char **argv){
    ros::init(argc, argv, "position_diff_calc");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    //declare input parameters
    string gps_topic_1;
    string gps_topic_2;
    string position_diff_topic;
    double gps_frequency_1;
    double gps_frequency_2;
    double position_diff_output_frequency;
    //load the parameters
    nh_priv.param<string>("gps_topic_1",gps_topic_1,"/n280/fix");
    nh_priv.param<string>("gps_topic_2",gps_topic_2,"/rtk/fix");
    nh_priv.param<string>("position_diff_topic",position_diff_topic,"/position_diff");
    nh_priv.param<double>("gps_frequency_1",gps_frequency_1,20.0);
    nh_priv.param<double>("gps_frequency_2",gps_frequency_2,10.0);
    nh_priv.param<double>("position_diff_output_frequency",position_diff_output_frequency,10.0);

    SensorInputGPS gps_1(nh,gps_topic_1,1.0/gps_frequency_1);
    SensorInputGPS gps_2(nh,gps_topic_2,1.0/gps_frequency_2);
    SensorOutputPositionDiff position_diff(nh,position_diff_topic,1.0/position_diff_output_frequency,gps_1,gps_2);

    ros::spin();
    return 0;
}
