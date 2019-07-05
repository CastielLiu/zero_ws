#include "lane_keeping/lane_keeping.h"
#include <assert.h>
#include <cmath>

LaneKeeping::LaneKeeping()
{
	cmd_.set_speed = 0.0;
	cmd_.set_steeringAngle = 0.0;
}

LaneKeeping::~LaneKeeping()
{

}

bool LaneKeeping::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<float>("foresight_distance",foresight_distance_,1.6);
	nh_private.param<float>("lane_keeping_speed",lane_keeping_speed_,1.0);
	nh_private.param<std::string>("cmd_topic_name",cmd_topic_name_,"/cmd");
	
	sub_laneMsg_ = nh.subscribe("/lane",1,&LaneKeeping::laneDetect_callback,this);
	
	pub_controlCmd_ = nh.advertise<driverless_msgs::ControlCmd>(cmd_topic_name_,2);
}

float polyval(const std::vector<float>& fit, float var)
{
	float result=0.0;
	for(int i=0; i<fit.size(); ++i)
		result += fit[i]*pow(var,fit.size()-1-i);
	return result;
}

void LaneKeeping::laneDetect_callback(const driverless_msgs::Lane::ConstPtr& msg)
{
	static float lowSpeed = 1.0;
	float foresight_dis = foresight_distance_;
	
	if((!msg->left_lane_validity) || (!msg->right_lane_validity))
	{
		foresight_dis = 1.5;
		cmd_.set_speed = lowSpeed;
	}
	else
		cmd_.set_speed = lane_keeping_speed_;
	
	float left_point_x = polyval(msg->dis_fit_left,foresight_dis);
	float right_point_x = polyval(msg->dis_fit_right,foresight_dis);
	
	float target_point_x,
		 target_point_y = foresight_dis;
	
	if(msg->left_lane_validity && (!msg->right_lane_validity))
		target_point_x = left_point_x +0.75;
	else if(msg->right_lane_validity && (!msg->left_lane_validity))
		target_point_x = right_point_x - 0.75;
	else
		target_point_x = (left_point_x + right_point_x) /2;
	
	float L = sqrt(target_point_x*target_point_x+target_point_y*target_point_y);
	float delta = atan2(target_point_x,target_point_y);
	float steering_radius = 0.5*L/sin(delta);
	
	if(msg->leftRightAngle == true)
	{
		float tempRadius = -2.0;
		if(tempRadius < steering_radius)
			steering_radius = tempRadius;
		cmd_.set_speed = lowSpeed;
	}
	
	cmd_.set_steeringAngle = -saturationEqual(generateRoadwheelAngleByRadius(steering_radius),15.0) * g_steering_gearRatio;
	
	pub_controlCmd_.publish(cmd_);
	
	
	static int _count=0;
	if(_count%10==0)
	{
		ROS_INFO("target_point x:%f\ty:%f",target_point_x,target_point_y);
		ROS_INFO("delta:%.2f\t radius:%.2f\t angle:%.2f",rad2deg(delta),steering_radius,cmd_.set_steeringAngle);
	}
	++ _count;
	
	
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"lane_keeping_node");
	
	LaneKeeping lane_keeping;
	
	if(lane_keeping.init())
		ros::spin();
	
	
	return 0;

}




