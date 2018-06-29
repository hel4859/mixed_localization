//This project contains very few comments.
//More information is detailed in the "description" of "../package.xml".

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float64.h"
#include "gl8_msgs/GPTRA_MSG.h"
#include "gl8_msgs/Heading.h"
#include "gl8_msgs/GPGGA_MSG.h"
#include "gl8_msgs/VehicleIMU.h"
#include "gl8_msgs/VehicleSpeedFeedBack.h"
#include "std_msgs/Int8.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "global_parameter.h"
#include "map_frame.h"
#include "ekf_pose.h"
#include "utils.h"


using namespace std;
sensor_msgs::NavSatFix sensor_gps;
gl8_msgs::GPTRA_MSG gps_attitude;
double MAPFRAME_OBS_ERR = 0.008, OBS_YAW_ERR = 0.2 * M_PI / 180.0;
bool have_inited = false, have_inited_can =false, have_inited_gps = false, filtered_wait_out = false;
EKF_Pose fusion;
Eigen::Vector3d X;
ros::Publisher pub_gps, pub_yaw;
double t_can = 0.0, t_gps = 0.0, t_output = 0.0, TIME_DIFF = 0.0;
double curr_vel,curr_yaw_rate;
double output_frequency, encoder_input_frequency, gps_input_frequency, imu_input_frequency;
double get_time_now();
double get_last_time(double t1,double t2);
void state_callback(const gl8_msgs::VehicleIMUConstPtr &imu_in, const gl8_msgs::VehicleSpeedFeedBackConstPtr &vel_in);
void update_callback(const sensor_msgs::NavSatFixConstPtr &gps_in, const gl8_msgs::GPTRA_MSGConstPtr &attitude_in,const gl8_msgs::GPGGA_MSGConstPtr &raw_in);
void filter_callback(const ros::TimerEvent&);


int main(int argc, char **argv){
    ros::init(argc, argv, "filter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    //declare input parameters
    string encoder_input_topic;
    string gps_input_topic;
    string imu_input_topic;
    string gps_output_topic;
    string yaw_output_topic;
    string gps_raw_topic;
    string gps_attitude_topic;
    //load the parameters
    nh_priv.param<string>("encoder_input_topic", encoder_input_topic,"/vehicle/speed_feedback");
    nh_priv.param<string>("gps_input_topic",gps_input_topic,"/strong/fix");
    nh_priv.param<string>("gps_attitude_topic",gps_attitude_topic,"/strong/attitude");
    nh_priv.param<string>("gps_raw_topic",gps_raw_topic,"/strong/raw_data");
    nh_priv.param<string>("imu_input_topic",imu_input_topic,"/vehicle/imu");
    nh_priv.param<string>("gps_output_topic",gps_output_topic,"/gps_filtered");
    nh_priv.param<string>("yaw_output_topic",yaw_output_topic,"/yaw_filtered");
    nh_priv.param<double>("output_frequency",output_frequency,100);
    nh_priv.param<double>("gps_input_frequency",gps_input_frequency,10);
    nh_priv.param<double>("encoder_input_frequency",encoder_input_frequency,100);
    nh_priv.param<double>("imu_input_frequency",imu_input_frequency,100);
    nh_priv.param<double>("imu_angular_velocity_error",IMU_W_ERR,0.0015);
    nh_priv.param<double>("GPS_error_fix",MAPFRAME_OBS_ERR_FIX,0.08);
    nh_priv.param<double>("GPS_error_float",MAPFRAME_OBS_ERR_FLOAT,1.0);
    nh_priv.param<double>("GPS_error_single",MAPFRAME_OBS_ERR_SINGLE,3.0);
    nh_priv.param<double>("encoder_error",REAR_WHEEL_ENCODER_ERR,0.03);
    nh_priv.param<double>("GPS_yaw_estimation_error",OBS_YAW_ERR_FIX,10 * M_PI / 180.0);

    pub_gps = nh.advertise<sensor_msgs::NavSatFix>(gps_output_topic,10);
    pub_yaw = nh.advertise<gl8_msgs::Heading>(yaw_output_topic,10);

    message_filters::Subscriber<sensor_msgs::NavSatFix> sub_gps_fix(nh, gps_input_topic, 1);
    message_filters::Subscriber<gl8_msgs::GPTRA_MSG> sub_gps_attitude(nh, gps_attitude_topic, 1);
    message_filters::Subscriber<gl8_msgs::GPGGA_MSG> sub_gps_raw(nh, gps_raw_topic, 1);
    message_filters::Subscriber<gl8_msgs::VehicleIMU> sub_imu(nh, imu_input_topic, 1);
    message_filters::Subscriber<gl8_msgs::VehicleSpeedFeedBack> sub_vel(nh, encoder_input_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<gl8_msgs::VehicleIMU, gl8_msgs::VehicleSpeedFeedBack> PredictionPolicy;
    message_filters::Synchronizer<PredictionPolicy> state_sync(PredictionPolicy(10),sub_imu,sub_vel);
    state_sync.registerCallback(boost::bind(&state_callback, _1, _2));

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,  gl8_msgs::GPTRA_MSG, gl8_msgs::GPGGA_MSG> UpdatePolicy;
    message_filters::Synchronizer<UpdatePolicy> update_sync(UpdatePolicy(10),sub_gps_fix,sub_gps_attitude,sub_gps_raw);
    update_sync.registerCallback(boost::bind(&update_callback, _1, _2, _3));

    ros::Timer filter_timer = nh.createTimer(ros::Duration(1.0/(output_frequency)), &filter_callback);

    ROS_INFO("EKF begin!");
    ros::spin();
    return 0;
}



void state_callback(const gl8_msgs::VehicleIMUConstPtr &imu_in, const gl8_msgs::VehicleSpeedFeedBackConstPtr &vel_in){
    if (!have_inited_gps){
        ROS_INFO("wait for GPS input!");
        return;
    }
    have_inited_can = true;
    t_can = get_time_now();
    curr_yaw_rate = ( imu_in->yaw_z+0.001043683)/1.00633477+0.0082;
   // curr_yaw_rate = imu_in->angular_velocity.z;
    curr_vel = vel_in->rear_wheel_speed;
}

double obs_yaw = 0;
void update_callback(const sensor_msgs::NavSatFixConstPtr &gps_in, const gl8_msgs::GPTRA_MSGConstPtr &attitude_in, const gl8_msgs::GPGGA_MSGConstPtr &raw_in){
    if (!have_inited_gps){
        have_inited_gps = true;
        TIME_DIFF = ros::Time::now().toSec() - gps_in->header.stamp.toSec();
        t_gps = get_time_now();
        return;
    }

    double t_1_gps = t_gps;
    t_gps = get_time_now();

    if (!have_inited_can){
        ROS_INFO("wait for can info");
        return;
    }

    int fix_type = raw_in->fix_type;
    if (isnan(gps_in->latitude)) return;
    switch (fix_type){
        case 1: MAPFRAME_OBS_ERR = MAPFRAME_OBS_ERR_SINGLE; break;
        case 4: MAPFRAME_OBS_ERR = MAPFRAME_OBS_ERR_FIX; break;
        case 5: MAPFRAME_OBS_ERR = MAPFRAME_OBS_ERR_FLOAT;break;
        default: return;
    }

    if (attitude_in->QF==4) OBS_YAW_ERR = OBS_YAW_ERR_FIX;
    else  OBS_YAW_ERR = M_PI;

    MapFrame obs_pos;
    obs_pos.GPS2MapFrame(*gps_in);
    //double obs_yaw = attitude_in->heading;
    if(attitude_in->heading < 10 && attitude_in->heading > -10) 
    {
	obs_yaw = attitude_in->heading;
	OBS_YAW_ERR = OBS_YAW_ERR_FIX;
    }
    else OBS_YAW_ERR = M_PI;
    if(obs_yaw >= M_PI) obs_yaw -= 2*M_PI;
    if(obs_yaw < -M_PI) obs_yaw += 2*M_PI;

    if (!have_inited){
        have_inited = true;
        Eigen::Matrix3d P0;
        X << obs_pos.x,obs_pos.y,obs_yaw;
        P0 << pow(MAPFRAME_OBS_ERR,2), 0, 0,
                0, pow(MAPFRAME_OBS_ERR,2), 0,
                0, 0, pow(OBS_YAW_ERR,2);
        fusion.init(X,P0);
        ROS_INFO("GPS ready!");
    }
    else{
        double dt = t_gps - get_last_time(t_1_gps,t_output);
        double d_dist = curr_vel * dt;
        double d_turn_rad = curr_yaw_rate * dt;
        if (!isZero(d_dist)) {
            filtered_wait_out = true;
            Eigen::Matrix2d Q;
            Q << pow(REAR_WHEEL_ENCODER_ERR * sqrt(dt), 2), 0,
                    0, pow(IMU_W_ERR * sqrt(dt), 2);
            X = fusion.statePrediction(d_dist, d_turn_rad, Q);
            Eigen::Matrix3d R;
            R << pow(MAPFRAME_OBS_ERR, 2), 0, 0,
                    0, pow(MAPFRAME_OBS_ERR, 2), 0,
                    0, 0, pow(OBS_YAW_ERR, 2);

            Eigen::Vector3d Z;
            Z << obs_pos.x, obs_pos.y, obs_yaw;
            X = fusion.stateUpdate(Z, R);
        }
    }
    sensor_gps = *gps_in;
    gps_attitude = *attitude_in;
}

void filter_callback(const ros::TimerEvent&){
    if(!have_inited or !have_inited_can) return;
    if (t_output-t_can > 5.0/encoder_input_frequency){
        ROS_INFO("can info loss! please restart this node");
        return;
    }
    double t_1_output = t_output;
    t_output = get_time_now();
    double dt = t_output - get_last_time(t_gps,t_1_output);
    double d_dist = curr_vel * dt;
    double d_turn_rad = curr_yaw_rate * dt;
    if (!isZero(d_dist)) {
        Eigen::Matrix2d Q;
        Q << pow(REAR_WHEEL_ENCODER_ERR * sqrt(dt), 2), 0,
               0, pow(IMU_W_ERR * sqrt(dt), 2);
        X = fusion.statePrediction(d_dist, d_turn_rad, Q);
    }
    MapFrame output_pos(X(0),X(1));
    double output_yaw_data = X(2);

    sensor_msgs::NavSatFix obs_gps = sensor_gps;
    gl8_msgs::Heading output_yaw;
    output_yaw.header.frame_id = "/yaw_filtered";
    output_yaw.header.stamp.fromSec(t_output);
    output_yaw.status = obs_gps.status;
    output_yaw.data = output_yaw_data;
    Eigen::Matrix3d P = fusion.readP();
    output_yaw.std_dev = sqrt(P(2,2));
    sensor_msgs::NavSatFix out_gps;
    out_gps = output_pos.MapFrame2GPS();

    out_gps.header.frame_id = "/gps_filtered";
    out_gps.header.stamp.fromSec(t_output);
    out_gps.status = obs_gps.status;

    if (filtered_wait_out) filtered_wait_out = false;
    else  out_gps.status.status = 6;

    pub_gps.publish(out_gps);
    pub_yaw.publish(output_yaw);
    MapFrame raw_pos;
    //double obs_yaw;
    //obs_yaw = gps_attitude.heading;
    raw_pos.GPS2MapFrame(obs_gps);
    double distance_diff = raw_pos.calcDistance(output_pos);
    double yaw_diff_rad = obs_yaw - output_yaw_data;
    double yaw_diff = yaw_diff_rad * 180.0 / M_PI;
    constrainDegree(yaw_diff);
    ROS_INFO(" \n\t\t\t dist difference(m): \t %f \n\t\t\t yaw difference (deg):\t %f \n\t\t\t yaw (deg):\t %f", distance_diff, yaw_diff, output_yaw.data * 180.0 / M_PI);
}

double get_time_now(){
    return ros::Time::now().toSec()-TIME_DIFF;
}

double get_last_time(double t1,double t2){
    return max(t1,t2);
}
