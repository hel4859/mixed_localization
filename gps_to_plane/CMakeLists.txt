cmake_minimum_required(VERSION 2.8.3)
project(gps_to_plane)


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        tf
        pcl_conversions
        pcl_ros
)


catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)

add_executable(gps_to_plane src/gps_to_plane.cpp)


target_link_libraries(gps_to_plane
  ${catkin_LIBRARIES}
 )
