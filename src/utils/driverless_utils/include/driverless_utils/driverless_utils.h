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


#define WHEEL_DISTANCE  0.4
#define AXIS_DISTANCE   0.7
#define MAX_SPEED 5.0


extern const float g_steering_gearRatio;
extern const float g_vehicle_width;
extern const float g_vehicle_length;

typedef struct
{
	double longitude;
	double latitude;
	float yaw;
}gpsMsg_t;


inline float generateRoadwheelAngleByRadius(const float& radius)
{
	assert(radius!=0);
	//return asin(AXIS_DISTANCE /radius)*180/M_PI;  //the angle larger
	return -atan(AXIS_DISTANCE/radius)*180/M_PI;    //correct algorithm 
}



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
float disBetween2Points(const gpsMsg_t &start, const gpsMsg_t &end);
std::pair<float, float> getDisAndYaw(const gpsMsg_t &start, const gpsMsg_t &end);

#endif


