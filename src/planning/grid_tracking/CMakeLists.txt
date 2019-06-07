cmake_minimum_required(VERSION 2.8.3)
project(grid_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gps_msgs
  serial
  driverless_msgs
  driverless_utils
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
#  CATKIN_DEPENDS little_ant_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
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

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node   ${catkin_LIBRARIES})

set(FileName reserve)

add_executable(${FileName}_node src/${FileName}.cpp)
add_dependencies(${FileName}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${FileName}_node   ${catkin_LIBRARIES})
