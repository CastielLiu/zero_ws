#ifndef GRID_TRACKING_H_
#define GRID_TRACKING_H_
#include<ros/ros.h>
#include<driverless_msgs/ControlCmd.h>
#include<driverless_utils/driverless_utils.h>
#include<std_msgs/Int8.h>
#include <limits.h>

//#include<little_ant_msgs/State2.h>  //speed
#include"gps_msgs/Inspvax.h"
#include<std_msgs/Bool.h>

#include<cmath>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<vector>
#include<thread>
#include<driverless_msgs/TrafficSign.h>


class GridTracking
{
public:
	GridTracking();
	~GridTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	void pub_cmd_callback(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void rosSpinThread();
	
	
private:
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_traffic_sign_;
	ros::Subscriber sub_vehicleState2_;
	
	ros::Timer timer_;
	
	ros::Publisher pub_cmd_;
	FILE * fp;
	
	std::string file_path_;
	gpsMsg_t *path_vertexes_ptr_;
    std::vector<gpsMsg_t> path_points_;
    
    boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	uint32_t target_point_index_;

    float disThreshold_;
	
	driverless_msgs::ControlCmd cmd_;
	
	float speed_;

	boost::mutex mutex_;
	bool is_gps_ok;
	

};


 




#endif

