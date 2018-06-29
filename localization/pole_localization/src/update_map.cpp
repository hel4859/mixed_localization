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

#include "gl8_msgs/Pole.h"
#include "gl8_msgs/PoleMap.h"


geometry_msgs::PoseStamped rtk_pose;
ros::Publisher pub_map;
ros::Publisher pub_markers;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExtracted(new pcl::PointCloud<pcl::PointXYZ>);
pcl::search::KdTree<pcl::PointXYZ> kdtree;
gl8_msgs::PoleMap poleMap;
gl8_msgs::PoleMap predPoleMap;

double curr_odom = 0.0, odom_inc=0.0;
bool first_time = true;

visualization_msgs::Marker createPoleMarker(const int& id, const pcl::PointXYZ& pt)
{
    visualization_msgs::Marker pole_marker;
    pole_marker.header.frame_id = "map";
    pole_marker.header.stamp = ros::Time();
    pole_marker.ns = "pole";
    pole_marker.id = id;
    pole_marker.type = visualization_msgs::Marker::CYLINDER;
    pole_marker.action = visualization_msgs::Marker::ADD;
    pole_marker.scale.x = 0.25;
    pole_marker.scale.y = 0.25;
    pole_marker.color.a = 0.6;
    pole_marker.color.r = 1.0;
    pole_marker.color.g = 1.0;
    pole_marker.color.b = 1.0;
    pole_marker.frame_locked = false;

    pole_marker.pose.position.x = pt.x;
    pole_marker.pose.position.y = pt.y;
    pole_marker.pose.position.z = pt.z / 2.0;
    pole_marker.pose.orientation.x = 0.0;
    pole_marker.pose.orientation.y = 0.0;
    pole_marker.pose.orientation.z = 0.0;
    pole_marker.pose.orientation.w = 1.0;
    pole_marker.scale.z = pt.z;
    pole_marker.lifetime = ros::Duration();

    return pole_marker;
}

gl8_msgs::Pole createPole(const int& id, const pcl::PointXYZ& pt)
{
    gl8_msgs::Pole p;
    p.id = id;
    p.x = pt.x;
    p.y = pt.y;
    p.h = pt.z;
    return p;
}


void rtkPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    if(!rtk_pose.header.frame_id.empty())
    {
        odom_inc = sqrtf( pow(msg.pose.position.x - rtk_pose.pose.position.x, 2) + pow(msg.pose.position.y - rtk_pose.pose.position.y, 2) );
        curr_odom += odom_inc;
    }
    rtk_pose = msg;

    if(curr_odom>20.0 || first_time){

        first_time = false;

        std::cout<<"curr_odom: "<<curr_odom<<std::endl;

        curr_odom = 0.0;

        // This vector will store the output neighbors.
//        std::vector<int> pointIndices;
        pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);
        std::vector<float> squaredDistances;

        pcl::PointXYZ point;
        point.x = rtk_pose.pose.position.x;
        point.y = rtk_pose.pose.position.y;
        point.z = rtk_pose.pose.position.z;

        // Now we find all neighbors within 3cm of the point
        // (inside a sphere of radius 50m centered at the point).
        if (kdtree.radiusSearch(point, 50, pointIndices->indices, squaredDistances) > 0)
        {
            visualization_msgs::MarkerArray poleMarker;
            predPoleMap.pole.clear();

            for(int i=0; i<pointIndices->indices.size(); ++i){
                poleMarker.markers.push_back(  createPoleMarker(i,cloud_map->points[ pointIndices->indices[i] ])  );
                predPoleMap.pole.push_back(poleMap.pole[ pointIndices->indices[i] ]);
            }

            pub_markers.publish(poleMarker);
            pub_map.publish(predPoleMap);

            std::cout<<"extract map"<<std::endl;

        }
    }


}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "update_map_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_rtk_filtered = nh.subscribe("filter", 10, rtkPoseCallback);
    pub_map = nh.advertise<gl8_msgs::PoleMap>("pred_pole_map",10);
    pub_markers = nh.advertise<visualization_msgs::MarkerArray>("pole_markers",10);



    // load pcd file
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/cyber/gl8_ws/src/map_manager/pole_mapping/map/campus_pole.pcd", *cloud_map)==-1) {
        std::cout << "load map failed!" << std::endl;
        return -1;
    }
    std::cout << "map loaded!" << std::endl;

    for(int i=0; i<cloud_map->points.size(); ++i)
        poleMap.pole.push_back( createPole(i, cloud_map->points[i]));

    kdtree.setInputCloud(cloud_map);


    ros::spin();
    return 0;
}
