#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_
#include<ros/ros.h>
#include<driverless_msgs/ControlCmd.h>
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

typedef struct
{
	double longitude;
	double latitude;
	float yaw;
}gpsMsg_t;




class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	float deg2rad(float deg);
	void limitRoadWheelAngle(float& angle);
	
	float point2point_dis(gpsMsg_t &point1,gpsMsg_t &point2);
	std::pair<float, float>  get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2);
	
	void pub_gps_cmd_callback(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	//void vehicleSpeed_callback(const zero_msgs::State2::ConstPtr& msg);
	//void avoiding_flag_callback(const std_msgs::Int8::ConstPtr& msg);
	void rosSpinThread();
	
	
private:
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_avoiding_from_lidar_;
	
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
	float vehicle_axis_dis_;
	float avoiding_disThreshold_;
	
	driverless_msgs::ControlCmd gps_controlCmd_;
	
	float speed_;

	boost::mutex mutex_;
	bool is_gps_ok;

    bool load_path_points(std::string file_path,std::vector<gpsMsg_t>& points)
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

};


 




#endif
