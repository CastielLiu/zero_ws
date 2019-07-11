#ifndef DECISION_MAKING_H_
#define DECISION_MAKING_H_
#include <ros/ros.h>
#include<driverless_msgs/ControlCmd.h>
#include<cmath>
#include<std_msgs/Bool.h>


#define SENSOR_NUM 2

struct
{
	bool status;
	float set_speed;
	float set_steeringAngle;
}cmdMsg_[SENSOR_NUM];




































#endif
