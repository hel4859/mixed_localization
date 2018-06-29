cmake_minimum_required(VERSION 2.8.3)
project(pose_fusion)

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

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
# Use ${EIGEN3_...} variables in either case below
roslint_cpp()


catkin_package(
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
  LIBRARIES
    global_parameter
    ekf_pose
    map_frame
    ${Eigen3_LIBRARIES}
    ${catkin_LIBRARIES}
  CATKIN_DEPENDS
    cmake_modules
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    gl8_msgs
  DEPENDS
    Eigen
    tf
)

###########
## Build ##
###########

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(filter src/main.cpp src/map_frame.cpp src/global_parameter.cpp src/ekf_pose.cpp src/utils.cpp)
target_link_libraries(filter ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

