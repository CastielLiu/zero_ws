#ifndef LANE_KEEPING_H_
#define LANE_KEEPING_H_
#include<ros/ros.h>
#include<driverless_msgs/Lane.h>
#include<driverless_msgs/ControlCmd.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<driverless_utils/driverless_utils.h>

#include<vector>

class LaneKeeping
{
public:
	LaneKeeping();
	~LaneKeeping();
	
	bool init();
	void laneDetect_callback(const driverless_msgs::Lane::ConstPtr& msg);
	
	
private:
	ros::Subscriber sub_laneMsg_;
	
	ros::Publisher pub_controlCmd_;
	
	float foresight_distance_;
	float lane_keeping_speed_;
	
	driverless_msgs::Lane lane_msg_;
	driverless_msgs::ControlCmd cmd_;
	
	std::string cmd_topic_name_;

};

#endif
