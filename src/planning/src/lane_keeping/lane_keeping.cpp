#include "lane_keeping/lane_keeping.h"
#include <assert.h>
#include <cmath>

LaneKeeping::LaneKeeping()
{
	cmd_.speed = 0.0;
	cmd_.steering_angle = 0.0;
}

LaneKeeping::~LaneKeeping()
{

}

bool LaneKeeping::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<float>("foresight_distance",foresight_distance_,2.0);
	nh_private.param<float>("lane_keeping_speed",lane_keeping_speed_,1.0);
	
	sub_laneMsg_ = nh.subscribe("/lane",1,&LaneKeeping::laneDetect_callback,this);
	
	pub_controlCmd_ = nh.advertise<driverless_msgs::ControlCmd>("/cmd",2);
}

void LaneKeeping::laneDetect_callback(const driverless_msgs::Lane::ConstPtr& msg)
{
	if(!msg->validity)
	{
		pub_controlCmd_.publish(cmd_);
		return ;
	}
	
	float alpha = asin(msg->offset *cos(msg->theta) / foresight_distance_) ;
	
	float delta = fabs(msg->theta) - fabs(alpha);
	
	//ROS_INFO("alpha:%f\t theta:%f \t delta:%f",alpha*180.0/M_PI,lane_msg_.included_angle*180.0/M_PI,delta*180.0/M_PI);
	
	float steering_radius = 0.5*foresight_distance_ / sin(delta) ;
	
	float angle = saturationEqual(generateRoadwheelAngleByRadius(steering_radius),15.0) * g_steering_gearRatio;
		
	cmd_.steering_angle = fabs(angle) * get_steeringDir(msg->offset, msg->theta,alpha);
	
	cmd_.speed = lane_keeping_speed_;
	
	pub_controlCmd_.publish(cmd_);
	
}


//left -1 ; right 1
int LaneKeeping::get_steeringDir(float err,float theta,float alpha)
{
	if(err==0 && theta==0)
		return 0;
	else if(err<=0 && theta<=0)
		return -1;
	else if(err>=0 && theta>=0)
		return 1;
	else if(err<0)
		return fabs(theta) > fabs(alpha) ? 1:-1;
	else if(err>0)
		return fabs(theta) > fabs(alpha) ? -1:1;
	else
		return 0;
}




int main(int argc, char** argv)
{
	ros::init(argc,argv,"lane_keeping_node");
	
	LaneKeeping lane_keeping;
	
	if(lane_keeping.init())
		ros::spin();
	
	
	return 0;

}




