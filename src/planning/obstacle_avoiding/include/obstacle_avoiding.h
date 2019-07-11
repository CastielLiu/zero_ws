#ifndef OBSTACLE_AVOIDING_H_
#define OBSTACLE_AVOIDING_H_
#include <ros/ros.h>
#include<driverless_msgs/lidardata.h>
#include<driverless_msgs/ControlCmd.h>
#include<cmath>
#include<std_msgs/Bool.h>
#define RAD2DEG(x) ((x)*180.0/M_PI)

float avoiding_disThreshold_;
bool Lidar_valid;
float Min_distance;
float Min_distance_degree;
float speed_;
float steeringAngle_;







#endif
