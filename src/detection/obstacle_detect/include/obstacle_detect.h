#ifndef OBSTACLE_DETECT_H_
#define OBSTACLE_DETECT_H_
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include<driverless_msgs/lidardata.h>
#include<cmath>
#include<std_msgs/Bool.h>
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.0)

bool lidar_valid;
float min_distance;
float min_distance_degree;





#endif
