#ifndef GRID_TRACKING_H_
#define GRID_TRACKING_H_
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

#define ROWS 4
#define COLS 2
#define DIS_INCREMENT 0.1
#define DIR_INVALID 255

struct vertexIndex_t
{
	uint8_t row;
	uint8_t col;
	
	bool equal_to(const vertexIndex_t& index)
	{
		if(this->row== index.row && this->col==index.col)
			return true;
		return false;
	}
};

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
	
	bool loadPathVertexes(const std::string& file_path);
	void generateNewPathPoints(uint8_t& traffic_dir, std::vector<gpsMsg_t>& path_points);
	bool generateStartPathPoints(std::vector<gpsMsg_t>& path_points);
	bool generateEndPathPoints(std::vector<gpsMsg_t>& path_points);
	
	uint8_t generateCurrentDir(const vertexIndex_t& target_index, const vertexIndex_t& last_index);
	bool updateTargetVertexIndex(uint8_t current_dir, uint8_t traffic_dir, vertexIndex_t& target_vertex_index);
	bool dumpPathPoints(const std::string& file_path, const std::vector<gpsMsg_t>& path_points);
	void rosSpinThread(){ros::spin();}
	
private:
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_traffic_sign_;
	ros::Subscriber sub_dump_;
	
	ros::Timer timer_;
	ros::Publisher pub_cmd_;
	
	std::string file_path_;
	gpsMsg_t path_vertexes_[ROWS][COLS];
    std::vector<gpsMsg_t> path_points_;
    
    boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	gpsMsg_t starting_point_;
	gpsMsg_t ending_point_;
	
	vertexIndex_t end_vertex_index_;
	
	uint32_t target_point_index_;
	
	vertexIndex_t target_vertex_index_, last_vertex_index_;
	
	bool is_trafficLightGreen_;
	
	uint8_t traffic_dir_;

    float disThreshold_;
	
	driverless_msgs::ControlCmd cmd_;
	
	float tracking_speed_;

	boost::mutex mutex_;
	bool is_gps_ok;
	

};
#endif

