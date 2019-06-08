#ifndef DRIVERLESS_UTILS_H_
#define DRIVERLESS_UTILS_H_

#include<cstring>
#include<cmath>
#include<assert.h>
#include<string>
#include<vector>
#include<cstdio>
#include<ros/ros.h>
#include<limits.h>
#include<exception>
#include"vehicle_params.h"


typedef struct
{
	double longitude;
	double latitude;
	float yaw;
}gpsMsg_t;


float generateRoadwheelAngleByRadius(const float& radius);


inline double sinDeg(const double& deg)
{
	return sin(deg*M_PI/180.0);
}

inline float saturationEqual(float value,float limit)
{
	//ROS_INFO("value:%f\t limit:%f",value,limit);
	assert(limit>=0);
	if(value>limit)
		value = limit;
	else if(value < -limit)
		value = -limit;
	return value;
}

inline int sign(float num)
{
	return num > 0? 1 : -1;
}

inline float deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

inline float rad2deg(float rad)
{
	return  rad*180.0/M_PI;
}

template <typename T>
inline void saturate_cast(T& value,const T& limit)
{
	if(value>limit)
		value = limit;
	else if(value<-limit)
		value = -limit;
}

bool load_path_points(const std::string& file_path,std::vector<gpsMsg_t>& points);

float limitRoadwheelAngleBySpeed(const float& angle, const float& speed);
float limitSpeedByPathCurvature(const float& speed,const float& curvature);

#define CONSIDER_DIR true
#define NON_DIR false
float disBetween2Points(const gpsMsg_t &start, const gpsMsg_t &end, bool orientation=false);
std::pair<float, float> getDisAndYaw(const gpsMsg_t &start, const gpsMsg_t &end, bool orientation=false);

#endif


