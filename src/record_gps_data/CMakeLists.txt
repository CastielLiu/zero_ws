cmake_minimum_required(VERSION 2.8.3)
project(record_gps_data)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gps_msgs
  serial
  
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_gps_data
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



add_executable(record_gps_data_node src/record_gps_data.cpp)
add_dependencies(record_gps_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_gps_data_node   ${catkin_LIBRARIES})

add_executable(trigger_record_node src/trigger_record.cpp)
add_dependencies(trigger_record_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trigger_record_node   ${catkin_LIBRARIES})

add_executable(show_gps_data_node src/show_gps_data.cpp)
add_dependencies(show_gps_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_gps_data_node   ${catkin_LIBRARIES})

