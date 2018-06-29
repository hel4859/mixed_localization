//
// Created by wlh on 17-7-30.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>

#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>

#include "gl8_msgs/Pole.h"
#include "gl8_msgs/PoleMap.h"
#include "gl8_msgs/PoleObservation.h"
#include "gl8_msgs/DeadReckoning.h"
#include "gl8_msgs/VehicleIMU.h"
#include "gl8_msgs/VehicleSpeedFeedBack.h"

#include "particle_filter/particle_filter.h"

#define DEBUG_PF  1


ParticleFilter pf;
Map map;
geometry_msgs::PoseStamped gt_pose;
gl8_msgs::DeadReckoning cur_deadreckoning, pre_deadreckoning;

ros::Publisher pub_particle;
ros::Publisher pub_pf_pose;
ros::Publisher pub_gt_pose;
ros::Publisher pub_pf_pose_with_cov;
ofstream eval;

//Set up parameters here
double delta_t = 0.1; // Time elapsed between measurements [sec]
double last_lidar_t = 0.0;
double sensor_range = 50; // Sensor range [m]

double sigma_pos [3] = {0.1, 0.1, 0.001}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
double sigma_landmark [2] = {0.12, 0.12}; // Landmark measurement uncertainty [x [m], y [m]]

double previous_velocity = 0.0;
double previous_yawrate = 0.0;

void mapToLocal(const double& ref_x, const double& ref_y, const double& ref_yaw,
                const double& x, const double& y, const double& yaw,
                double& local_x, double& local_y, double& local_yaw)
{
    double dx = x - ref_x;
    double dy = y - ref_y;
    local_x = dx * cos(ref_yaw) + dy * sin(ref_yaw);
    local_y = dx * -sin(ref_yaw) + dy * cos(ref_yaw);
    local_yaw = yaw - ref_yaw;
}

void predMapCallback(gl8_msgs::PoleMap pole_map)
{
//    std::cout<<"pred map size: "<<pole_map.pole.size()<<std::endl;
    for(int i=0; i<pole_map.pole.size(); ++i){
        Map::single_landmark_s lm;
        lm.id_i = pole_map.pole[i].id;
        lm.x_f = pole_map.pole[i].x;
        lm.y_f = pole_map.pole[i].y;
        map.landmark_list.push_back(lm);
    }
}

void rtkPoseCallback(geometry_msgs::PoseStamped rtk_pose)
{
    gt_pose = rtk_pose;
    if (!pf.initialized()) {

        double sense_x = gt_pose.pose.position.x;
        double sense_y = gt_pose.pose.position.y;
        double sense_theta = tf::getYaw(gt_pose.pose.orientation);

        pf.init(sense_x, sense_y, sense_theta, sigma_pos);
    }
//    std::cout<<"rtk t:"<<rtk_pose.header.stamp.toSec()<<"\tx:"<<rtk_pose.pose.position.x<<"\ty: "<<rtk_pose.pose.position.y<<"\ttheta: "<<tf::getYaw(rtk_pose.pose.orientation)<<std::endl;
}

void observationCallback(gl8_msgs::PoleObservation obs_in)
{
    if (!pf.initialized() || (int)last_lidar_t==0 ) {
        last_lidar_t = obs_in.header.stamp.toSec();
        return;
    }
    else {
        // Predict the vehicle's next state from previous (noiseless control) data.

        delta_t = obs_in.header.stamp.toSec() - last_lidar_t;

//      std::cout<<"delta_t: "<<delta_t<<"\tds: "<<ds<<"\tdyaw: "<<dtheta<<"\tdt: "<<dt<<"\tv: "<<previous_velocity<<"\tyaw_rete: "<<previous_yawrate<<std::endl;

        int last_best = pf.best_particle.id;
        double last_x = pf.particles[last_best].x;
        double last_y = pf.particles[last_best].y;
        double last_yaw = pf.particles[last_best].theta;

        if(!obs_in.pole.empty())
            pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
        else
            pf.deadReckoning(delta_t, previous_velocity, previous_yawrate);

        int pred_best = pf.best_particle.id;
        double pred_x = pf.particles[pred_best].x;
        double pred_y = pf.particles[pred_best].y;
        double pred_yaw = pf.particles[pred_best].theta;

        if(std::isnan(pred_x) || std::isnan(pred_y) || std::isnan(pred_yaw))
        {
            pf.reset(last_x, last_y, last_yaw);
            std::cout<<"reset pf to "<<last_x<<" "<<last_y<<" "<<last_yaw<<std::endl;
        }
    }

    std::vector<LandmarkObs> noisy_observations;
    for(int i = 0; i < obs_in.pole.size(); i++)
    {
        LandmarkObs obs;
        obs.id = obs_in.pole[i].id;
        obs.x = obs_in.pole[i].x;
        obs.y = obs_in.pole[i].y;
        noisy_observations.push_back(obs);
//        std::cout<<"obs_id: "<<obs.id<<"\tobsx: "<<obs.x<<"\tobsy: "<<obs.y<<std::endl;
    }

    int pred_best = pf.best_particle.id;
    double pred_x = pf.particles[pred_best].x;
    double pred_y = pf.particles[pred_best].y;
    double pred_yaw = pf.particles[pred_best].theta;
//    std::cout<<"pred_x: "<<pred_x<<"\tpred_y: "<<pred_y<<"\tpred_yaw: "<<pred_yaw<<std::endl;

    if(!obs_in.pole.empty()){
        // Update the weights and resample
        pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
        pf.resample();

        double update_x = pf.particles[pred_best].x;
        double update_y = pf.particles[pred_best].y;
        double update_yaw = pf.particles[pred_best].theta;
//        std::cout<<"update_x: "<<update_x<<"\tupdate_y: "<<update_y<<"\tupdate_yaw: "<<update_yaw<<std::endl;
        if(fabs(update_x-pred_x)>10 || fabs(update_y-pred_y)>10)
        {
            pf.reset(pred_x, pred_y, pred_yaw);
            std::cout<<"reset pf to "<<pred_x<<" "<<pred_y<<" "<<pred_yaw<<std::endl;
        }
    }

//    std::cout<<"observation size: "<<obs_in.pole.size()<<std::endl;

    double delay_t = gt_pose.header.stamp.toSec() - obs_in.header.stamp.toSec();
    pf.processDelay(delay_t,previous_velocity,previous_yawrate);
    last_lidar_t = gt_pose.header.stamp.toSec();

//    std::cout<<"time now: "<<std::setprecision(14)<<ros::Time::now().toSec()<<std::endl;
//    std::cout<<"latest rtk time: "<<std::setprecision(14)<<gt_pose.header.stamp.toSec()<<std::endl;
//    std::cout<<"latest obs time: "<<std::setprecision(14)<<obs_in.header.stamp.toSec()<<std::endl;
//    std::cout<<"delay time: "<<std::setprecision(14)<<delay_t<<std::endl;
//    std::cout<<"pre velocity: "<<previous_velocity<<std::endl;
//    std::cout<<"delay distance: "<<delay_t*previous_velocity<<std::endl;

    geometry_msgs::PoseArray particle_array;
    geometry_msgs::Pose p;
    double highest_weight = -1.0;
    double weight_sum = 0.0;
    Particle best_particle;

    for(int i=0; i<pf.particles.size(); ++i){

        if (pf.particles[i].weight > highest_weight) {
            highest_weight = pf.particles[i].weight;
            best_particle = pf.particles[i];
        }
        weight_sum += pf.particles[i].weight;

        p.position.x = pf.particles[i].x;
        p.position.y = pf.particles[i].y;
        p.position.z = 0.0;
        p.orientation = tf::createQuaternionMsgFromYaw(pf.particles[i].theta);
        particle_array.poses.push_back(p);
    }
    particle_array.header.stamp = ros::Time::now();
    particle_array.header.frame_id = "map";
    pub_particle.publish(particle_array);
    pf.best_particle = best_particle;

//    std::cout<<"best_x: "<<best_particle.x<<"best_y: "<<best_particle.y<<"best_yaw: "<<best_particle.theta<<std::endl;

//    cout << "highest w " << highest_weight << endl;
//    cout << "average w " << weight_sum/pf.particles.size() << endl;

    //pub pf pose
    geometry_msgs::PoseStamped pf_pose;
    pf_pose.header.stamp = particle_array.header.stamp;
    pf_pose.header.frame_id = "map";
    pf_pose.pose.position.x = best_particle.x;
    pf_pose.pose.position.y = best_particle.y;
    pf_pose.pose.position.z = 0;
    pf_pose.pose.orientation = tf::createQuaternionMsgFromYaw(best_particle.theta);
    pub_pf_pose.publish(pf_pose);

    geometry_msgs::PoseWithCovarianceStamped pf_pose_cov;
    pf_pose_cov.header = pf_pose.header;
    pf_pose_cov.pose.pose = pf_pose.pose;
    pf_pose_cov.pose.covariance.data()[0] = obs_in.pole.size();
    pub_pf_pose_with_cov.publish(pf_pose_cov);


#if DEBUG_PF
    //send pf tf
    static tf::TransformBroadcaster br_pf;
    tf::Transform transform_pf;
    tf::Quaternion q_pf;
    transform_pf.setOrigin(tf::Vector3(best_particle.x, best_particle.y, 0));
    q_pf.setRPY(0, 0, best_particle.theta);
    transform_pf.setRotation(q_pf);
    br_pf.sendTransform( tf::StampedTransform(transform_pf, gt_pose.header.stamp, "map" ,"pf"));

    //pub gt pose
    pub_gt_pose.publish(gt_pose);

    //send gt tf
    static tf::TransformBroadcaster br_gt;
    tf::Transform transform_gt;
    tf::Quaternion q_gt;
    transform_gt.setOrigin(tf::Vector3(gt_pose.pose.position.x, gt_pose.pose.position.y, 0));
    q_gt.setRPY(0, 0, tf::getYaw(gt_pose.pose.orientation));
    transform_gt.setRotation(q_gt);
    br_gt.sendTransform( tf::StampedTransform(transform_gt, gt_pose.header.stamp, "map" ,"gt"));

    double x_diff, y_diff, yaw_diff;
    mapToLocal(gt_pose.pose.position.x, gt_pose.pose.position.y, tf::getYaw(gt_pose.pose.orientation),
                pf_pose.pose.position.x, pf_pose.pose.position.y, tf::getYaw(pf_pose.pose.orientation),
                x_diff, y_diff, yaw_diff);

    if( fabs(x_diff)>0.8 || fabs(y_diff)>0.8 )
        pf.reset(gt_pose.pose.position.x, gt_pose.pose.position.y, tf::getYaw(pf_pose.pose.orientation) );
#endif
}

void deadReckoningCallback(gl8_msgs::DeadReckoning msg)
{
    if(cur_deadreckoning.header.seq == 0){
        cur_deadreckoning = msg;
        pre_deadreckoning = msg;
        cur_deadreckoning.header.seq = 1;
        pre_deadreckoning.header.seq = 1;
        return;
    }
    cur_deadreckoning = msg;
}

void imuCallback(const gl8_msgs::VehicleIMUConstPtr &imu_in)
{
    previous_yawrate = imu_in->yaw_z;
}

void speedFeedbackCallback(const gl8_msgs::VehicleSpeedFeedBackConstPtr &vel_in)
{
    previous_velocity = vel_in->rear_wheel_speed;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pf_localization_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_pred_map = nh.subscribe("pred_pole_map", 10, predMapCallback);
    ros::Subscriber sub_rtk_pose = nh.subscribe("filter", 10, rtkPoseCallback);
    ros::Subscriber sub_observation = nh.subscribe("pole_observation", 10, observationCallback);
    ros::Subscriber sub_speed = nh.subscribe("/vehicle/speed_feedback", 10, speedFeedbackCallback);
    ros::Subscriber sub_imu = nh.subscribe("/vehicle/imu", 10, imuCallback);

    pub_particle = nh.advertise<geometry_msgs::PoseArray>("particles", 10);
    pub_pf_pose = nh.advertise<geometry_msgs::PoseStamped>("pf_pose", 10);
    pub_gt_pose = nh.advertise<geometry_msgs::PoseStamped>("gt_pose", 10);
    pub_pf_pose_with_cov = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pf_pose_with_cov", 10);


    ros::spin();
    return 0;
}
