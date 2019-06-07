#ifndef RESERVE_H_
#define RESERVE_H_
#include<ros/ros.h>
#include<driverless_msgs/ControlCmd.h>
#include<driverless_utils/driverless_utils.h>
#include<std_msgs/String.h>
#include<std_msgs/Int8.h>
#include <limits.h>

#include"gps_msgs/Inspvax.h"
#include<std_msgs/Bool.h>

#include<cmath>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<vector>
#include<thread>
#include<driverless_msgs/TrafficSign.h>


#define DIS_INCREMENT 0.1


class GridTracking
{
public:
	GridTracking();
	~GridTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	void cmd_timer(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void traffic_sign_callback(const driverless_msgs::TrafficSign::ConstPtr &msg);
	void dump_callback(const std_msgs::String::ConstPtr& msg);
	
	bool loadPathVertexes(const std::string& file_path, std::vector<gpsMsg_t>& path_vertexes);

	bool generatePathPoints(std::vector<gpsMsg_t>& path_points);
	
	bool dumpPathPoints(const std::string& file_path, const std::vector<gpsMsg_t>& path_points);
	void rosSpinThread(){ros::spin();}
	
private:
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_traffic_sign_;
	ros::Subscriber sub_dump_;
	
	ros::Timer timer_;
	ros::Publisher pub_cmd_;
	
	std::string file_path_;
	
	std::vector<gpsMsg_t> path_points_;
	std::vector<gpsMsg_t> path_vertexes_;
	
	size_t target_point_index_;
		
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	bool is_trafficLightGreen_;
	
    float disThreshold_;
	
	driverless_msgs::ControlCmd cmd_;
	
	float tracking_speed_;

	boost::mutex mutex_;
	bool is_gps_ok;
	
};
#endif

