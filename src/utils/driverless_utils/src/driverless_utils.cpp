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

float disBetween2Points(const gpsMsg_t &start, const gpsMsg_t &end)
{
	float x = (end.longitude -start.longitude)*111000*cos(end.latitude*M_PI/180.0);
	float y = (end.latitude - start.latitude ) *111000;
	
	float dis = sqrt(x * x + y * y);
	float yaw = atan2(x,y) *180.0/M_PI;
	if(yaw <0)
		yaw += 360.0;
		
	if(fabs(yaw- start.yaw)>90.0)
		dis *= -1;
	return dis;
}

std::pair<float, float> getDisAndYaw(const gpsMsg_t &start, const gpsMsg_t &end)
{
	float x = (end.longitude -start.longitude)*111000*cos(end.latitude*M_PI/180.0);
	float y = (end.latitude - start.latitude ) *111000;
	
	std::pair<float, float> dis_yaw;
	
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(x,y) *180.0/M_PI;
	if(dis_yaw.second <0)
		dis_yaw.second += 360.0;
		
	if(fabs(dis_yaw.second - start.yaw)>90.0)
		dis_yaw.first *= -1;
	return dis_yaw;
}

 bool load_path_points(const std::string& file_path,std::vector<gpsMsg_t>& points)
 {
	FILE *fp = fopen(file_path.c_str(),"r");
	
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return false;
	}
	
	gpsMsg_t point;
	
	while(!feof(fp))
	{

		fscanf(fp,"%lf\t%lf\t",&point.longitude,&point.latitude);
			
		points.push_back(point);
	}
	fclose(fp);
	
	return true;
 }


