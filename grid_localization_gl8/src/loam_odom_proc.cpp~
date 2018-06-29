

#include "loam_odom_proc.h"

using namespace std;
using namespace mrpt;


void odom_callback(const nav_msgs::Odometry& msg)
{
    double pose_x = msg.pose.pose.position.x;
    double pose_y = msg.pose.pose.position.y;
    double pose_z = msg.pose.pose.position.z;

    double x = msg.pose.pose.orientation.x;
    double y = msg.pose.pose.orientation.y;
    double z = msg.pose.pose.orientation.z;
    double w = msg.pose.pose.orientation.w;

    double roll = atan2(2*(w*z+x*y),1-2*(z*z+x*x));
    double pitch = asin(2*(w*x-y*z));
    double yaw = atan2(2*(w*y+z*y),1-2*(y*y+x*x));

    //ROS_INFO("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n",pose_x,pose_y,pose_z,roll,pitch,yaw);

    //output_file.printf("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n",pose_x,pose_y,pose_z,roll,pitch,yaw);
    
    //nav_msgs::Odometry odom_out;
    //pub_odom.publish(odom_out);

    std_msgs::Float64MultiArray odom_out;
    odom_out.data.push_back(pose_x);
    odom_out.data.push_back(pose_y);
    odom_out.data.push_back(pose_z);
    odom_out.data.push_back(roll);
    odom_out.data.push_back(pitch);
    odom_out.data.push_back(yaw);
    pub_odom.publish(odom_out);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "loam_odom_proc");

    ros::NodeHandle nh;
    sub_odom = nh.subscribe("/integrated_to_init",1,&odom_callback);
    pub_odom = nh.advertise<std_msgs::Float64MultiArray>("/odom",1);

    output_file.open("/home/hl/helei_ws/src/grid_localization_gl8/output.txt",0);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    ros::waitForShutdown();
    output_file.close();

    return 0;
}
