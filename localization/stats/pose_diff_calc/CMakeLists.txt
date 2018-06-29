cmake_minimum_required(VERSION 2.8.3)
project(pose_diff_calc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
  gl8_msgs
  message_generation
  tf)
# find_package(Boost REQUIRED COMPONENTS system)


roslint_cpp()


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    sensor
    global_parameter
    map_frame
    position_diff
    angle_diff
  CATKIN_DEPENDS
    cmake_modules
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    gl8_msgs
  DEPENDS
    tf
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(position_diff_calc src/position_diff.cpp src/global_parameter.cpp)
add_executable(angle_diff_calc src/angle_diff.cpp src/global_parameter.cpp)
add_executable(encoder_error_calc src/encoder_error_calc.cpp src/global_parameter.cpp)
add_executable(angle_error_calc src/angle_error_calc.cpp src/global_parameter.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(position_diff_calc ${catkin_LIBRARIES})
target_link_libraries(angle_diff_calc ${catkin_LIBRARIES})
target_link_libraries(encoder_error_calc ${catkin_LIBRARIES})
target_link_libraries(angle_error_calc ${catkin_LIBRARIES})
## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(position_diff_calc pose_diff_calc_generate_messages_gencpp)
#add_dependencies(angle_diff_calc pose_diff_calc_generate_messages_gencpp)
#add_dependencies(encoder_error_calc pose_diff_calc_generate_messages_gencpp)
#add_dependencies(angle_error_calc pose_diff_calc_generate_messages_gencpp)
