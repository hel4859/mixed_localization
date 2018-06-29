cmake_minimum_required(VERSION 2.8.3)
project(fix2pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  sensor_msgs
  visualization_msgs
  gl8_msgs
  gl8_utils
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs tf sensor_msgs visualization_msgs  gl8_msgs gl8_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

link_directories(
  lib
)

#add_library(map_frame src/map_frame.cpp)
#add_library(global_parameter src/global_parameter.cpp)
add_executable(fix2tfpose src/fix2tfpose.cpp src/map_frame.cpp src/global_parameter.cpp)
target_link_libraries(fix2tfpose ${catkin_LIBRARIES})

add_executable(filter2tfpose src/filter2tfpose.cpp src/map_frame.cpp src/global_parameter.cpp)
target_link_libraries(filter2tfpose ${catkin_LIBRARIES})

add_executable(point2tfpose src/point2tfpose.cpp src/map_frame.cpp src/global_parameter.cpp)
target_link_libraries(point2tfpose ${catkin_LIBRARIES})

add_executable(n280conv src/n280conv.cpp)
target_link_libraries(n280conv ${catkin_LIBRARIES})

add_executable(pose2path src/pose2path.cpp)
target_link_libraries(pose2path ${catkin_LIBRARIES})

add_executable(pose2path2 src/pose2path2.cpp)
target_link_libraries(pose2path2 ${catkin_LIBRARIES})
