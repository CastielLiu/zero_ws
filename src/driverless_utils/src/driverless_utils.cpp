#include "driverless_utils/driverless_utils.h"

#define MAX_STEERING_ANGLE 25.0
#define MAX_ROAD_WHEEL_ANGLE 25.0

//方向盘最大转角/前轮最大转角
const float g_steering_gearRatio = MAX_STEERING_ANGLE/MAX_ROAD_WHEEL_ANGLE;

const float g_vehicle_width = 0.45 ;// m
const float g_vehicle_length = 0.70; 

static const float max_side_acceleration = 1.9; // m/s/s

float limitRoadwheelAngleBySpeed(const float& angle, const float& speed)
{
	float min_steering_radius = speed*speed/max_side_acceleration;
	if(min_steering_radius <3.0)  
		min_steering_radius = 3.0;
	
	float max_roadwheelAngle = fabs(generateRoadwheelAngleByRadius(min_steering_radius));
	if(max_roadwheelAngle > MAX_ROAD_WHEEL_ANGLE - 5.0)
	   max_roadwheelAngle = MAX_ROAD_WHEEL_ANGLE -5.0;
	//ROS_INFO("max_angle:%f\t angle:%f",max_roadwheelAngle,angle);
	return saturationEqual(angle,max_roadwheelAngle);
}

float limitSpeedByCurrentRoadwheelAngle(float speed,float angle)
{
	float steering_radius = fabs(AXIS_DISTANCE/tan(angle*M_PI/180.0));
	float max_speed =  sqrt(steering_radius*max_side_acceleration);
	return speed>max_speed? max_speed: speed;
}



float limitSpeedByPathCurvature(const float& speed,const float& curvature)
{
	if(curvature == 0.0)
		return speed;
	
	//float max_speed =  sqrt(1.0/fabs(curvature)*max_side_acceleration) *3.6;
	float max_speed =  sqrt(1.0/fabs(curvature)*1.5) *3.6;
	return speed>max_speed? max_speed: speed;
}

