set(MAKE_THIS_PKG 0)

if(MAKE_THIS_PKG)

	cmake_minimum_required(VERSION 2.8.3)
	project(to_little_ant)

	## Compile as C++11, supported in ROS Kinetic and newer
	# add_compile_options(-std=c++11)

	find_package(catkin REQUIRED COMPONENTS
	  driverless_msgs
	  driverless_utils
	  gps_msgs
	  little_ant_msgs
	  sensor_msgs
	  std_msgs
	  driverless_utils
	)


	catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES to_little_ant
	#  CATKIN_DEPENDS driverless_msgs driverless_utils gps_msgs little_ant_msgs sensor_msgs std_msgs
	#  DEPENDS system_lib
	)


	include_directories(
	# include
	  ${catkin_INCLUDE_DIRS}
	)

	add_executable(cmd_transform_node src/cmd_transform.cpp)
	add_dependencies(cmd_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(cmd_transform_node ${catkin_LIBRARIES})

endif()
