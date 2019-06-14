cmake_minimum_required(VERSION 2.8.3)
project(base_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  serial
  sensor_msgs
  driverless_msgs
)

##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 #generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_node
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp )
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

