#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_
#include<ros/ros.h>
#include<driverless_msgs/ControlCmd.h>
#include<std_msgs/Int8.h>
#include <limits.h>
#include<driverless_utils/driverless_utils.h>
#include"gps_msgs/Inspvax.h"
#include<std_msgs/Bool.h>

#include<cmath>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<vector>
#include<thread>


class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	void pub_gps_cmd_callback(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void rosSpinThread();
	
	
private:
	ros::Subscriber sub_gps_;
	
	ros::Timer timer_;
	
	ros::Publisher pub_gps_cmd_;
	FILE * fp;
	
	std::string path_points_file_;
    std::vector<gpsMsg_t> path_points_;
    boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	uint32_t target_point_index_;

    float disThreshold_;
	
	driverless_msgs::ControlCmd gps_controlCmd_;
	
	float speed_;
	float low_speed_;

	boost::mutex mutex_;
	bool is_gps_ok;

};




#endif

