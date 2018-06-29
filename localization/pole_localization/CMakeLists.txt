cmake_minimum_required(VERSION 2.8.3)
project(pole_localization)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation 
	roscpp 
	std_msgs
	pcl_conversions
	pcl_ros
	gl8_msgs
	particle_filter
	kalman_filter
)
find_package(PCL 1.7 REQUIRED common io visualization)


catkin_package(
  	CATKIN_DEPENDS
  	message_runtime 
  	roscpp 
  	std_msgs
	particle_filter
	kalman_filter
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
    include/
)

# Set the name to use for the executable.
set (BINNAME1 update_map_node)
set (BINNAME2 pf_localization)

# Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/update_map.cpp)
set (SRCS2 ${SRCS2} src/pf_localization.cpp)

#Build the executable that will be used to run this node.
add_executable (${BINNAME1} ${SRCS1})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})

add_executable (${BINNAME2} ${SRCS2})
target_link_libraries(${BINNAME2} ${catkin_LIBRARIES})


#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

#install(TARGETS ${BINNAME1}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(TARGETS ${BINNAME2}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
