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

#define ROWS 4
#define COLS 2
#define DIS_INCREMENT 0.1
#define DIR_INVALID 255

class GridTracking
{
public:
	GridTracking();
	~GridTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	void pub_cmd_callback(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	bool loadPathVertexes(const std::string& file_path,  gpsMsg_t** path_vertexes);
	void generateNewPathPoints(uint8_t dir, std::vector<gpsMsg_t>& path_points, bool first_time=false);
	void rosSpinThread();
	
private:
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_traffic_sign_;
	
	ros::Timer timer_;
	ros::Publisher pub_cmd_;
	
	std::string file_path_;
	gpsMsg_t path_vertexes_[ROWS][COLS];
    std::vector<gpsMsg_t> path_points_;
    
    boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	uint32_t target_point_index_;
	
	struct 
	{
		uint8_t row;
		uint8_t col;
	}target_vertex_index_, last_vetex_index_;
	
	uint8_t traffic_light_;
	uint8_t traffic_dir_;

    float disThreshold_;
	
	driverless_msgs::ControlCmd cmd_;
	
	float speed_;

	boost::mutex mutex_;
	bool is_gps_ok;
	

};


 




#endif

