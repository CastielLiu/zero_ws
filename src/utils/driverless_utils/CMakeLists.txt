cmake_minimum_required(VERSION 2.8.3)
project(driverless_utils)

# Find catkin
find_package(catkin REQUIRED COMPONENTS 
	#serial
	roscpp
	rospy
	std_msgs
	driverless_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp rospy
	LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS include
	#CATKIN_DEPENDS serial #
)


## Sources
set(_SRCS
    src/driverless_utils.cpp
    include/driverless_utils/driverless_utils.h
)
 

add_library(${PROJECT_NAME} ${_SRCS})

#target_link_libraries(${PROJECT_NAME} boost)

## Include headers
include_directories(include ${catkin_INCLUDE_DIRS})

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/driverless_utils.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/driverless_utils)


