cmake_minimum_required(VERSION 2.8.3)
project(lane_detect)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	geometry_msgs 
	roscpp 
	tf 
	std_msgs 
	rospy 
	sensor_msgs 
	cv_bridge 
	image_transport
	dynamic_reconfigure
	param_config
)

catkin_package(
  CATKIN_DEPENDS
)

