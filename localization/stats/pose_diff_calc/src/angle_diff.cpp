//This project contains very few comments.
//More information is detailed in the "description" of "../package.xml".
#include "../include/angle_diff.h"
using namespace std;
int main(int argc, char **argv){
    ros::init(argc, argv, "angle_diff_calc");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    //declare input parameters
    string angle_topic_1;
    string angle_topic_2;
    string angle_diff_topic;
    double angle_frequency_1;
    double angle_frequency_2;
    double angle_diff_output_frequency;
    //load the parameters
    nh_priv.param<string>("angle_topic_1",angle_topic_1,"/n280/GPS_N280/orientation");
    nh_priv.param<string>("angle_topic_2",angle_topic_2,"yaw_filtered");
    nh_priv.param<string>("angle_diff_topic",angle_diff_topic,"/angle_diff");
    nh_priv.param<double>("angle_frequency_1",angle_frequency_1,20);
    nh_priv.param<double>("angle_frequency_2",angle_frequency_2,10);
    nh_priv.param<double>("angle_diff_output_frequency",angle_diff_output_frequency,10);

    SensorInputAngle angle_1(nh,angle_topic_1,1.0/angle_frequency_1);
    SensorInputAngle angle_2(nh,angle_topic_2,1.0/angle_frequency_2);
    SensorOutputAngleDiff angle_diff(nh,angle_diff_topic,1.0/angle_diff_output_frequency,angle_1,angle_2);

    ros::spin();
    return 0;
}
