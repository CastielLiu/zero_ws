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

void LaneKeeping::laneDetect_callback(const driverless_msgs::Lane::ConstPtr& msg)
{
	if(!msg->validity)
	{
		pub_controlCmd_.publish(cmd_);
		return ;
	}
	
	float alpha = asin(msg->offset *cos(msg->theta) / foresight_distance_);
	
	//float delta = getDelta(alpha,msg->theta);
	float delta = -msg->theta -alpha;
	
	float steering_radius = 0.5*foresight_distance_ / sin(delta) ;
	
	cmd_.set_steeringAngle = saturationEqual(generateRoadwheelAngleByRadius(steering_radius),15.0) * g_steering_gearRatio;
		
	cmd_.set_speed = lane_keeping_speed_;
	
	ROS_INFO("theta:%.2f\t alpha:%.2f\t delta:%.2f\t radius:%.2f\t angle:%.2f",
				rad2deg(msg->theta),rad2deg(alpha),rad2deg(delta),steering_radius,cmd_.set_steeringAngle);
	
	pub_controlCmd_.publish(cmd_);
	
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"lane_keeping_node");
	
	LaneKeeping lane_keeping;
	
	if(lane_keeping.init())
		ros::spin();
	
	
	return 0;

}




