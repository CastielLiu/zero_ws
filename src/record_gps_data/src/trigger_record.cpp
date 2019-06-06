#include<ros/ros.h>
#include "gps_msgs/Inspvax.h"
#include <unistd.h>
#include<cmath>

#include<keyEvent.h>
#include<boost/bind.hpp>
#include<boost/thread.hpp>

#ifndef PI_
#define PI_ 3.141592653589
#endif

typedef struct
{
	double latitude;
	double longitude;
}location_t;

class Record
{
	private:
		void gps_callback(const gps_msgs::Inspvax::ConstPtr& gpsMsg);
		void timerCallback(const ros::TimerEvent&);
		float calculate_dis2(location_t & point1,location_t& point2);
		
		std::string file_name_;
		FILE *fp;
		
		location_t last_point , current_point;
		
		KeyEvent keyEvent;
		
		float min_sample_distance_;
		ros::Subscriber gps_sub;
		bool is_gps_ok_;
		int system_delay_;
		
	public:
		Record();
		~Record() {fclose(fp);}
		bool init();
		void recordToFile()
		{
			ROS_INFO("trigger...");
			if(min_sample_distance_*min_sample_distance_ <= calculate_dis2(current_point,last_point))
			{
				fprintf(fp,"%.8f\t%.8f\r\n",current_point.longitude,current_point.latitude);
				fflush(fp);
				ROS_INFO("%.8f\t%.8f\r\n",current_point.longitude,current_point.latitude);
				last_point = current_point;
			}
		}
		void run();
		void ros_spin(){ros::spin();}
};

Record::Record():
	is_gps_ok_(false),
	system_delay_(20)
{
	last_point.longitude = 0.0;
	last_point.latitude = 0.0;
}


bool Record::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("file_name",file_name_,"");
	if(file_name_.empty())
	{
		ROS_ERROR("no input file !!");
		return false;
	}
	private_nh.param<float>("min_sample_distance",min_sample_distance_,0.1);
	
	gps_sub= nh.subscribe("/gps",1,&Record::gps_callback,this);

	fp = fopen(file_name_.c_str(),"w");
	
	if(fp == NULL)
	{
		ROS_INFO("open record data file failed !!!!!");
		return false;
	}
	
	if(!keyEvent.init(KeyEvent::ONLY_KEYBORD))
		return false;
	
	return true;
}

void Record::run()
{
	boost::shared_ptr<boost::thread> read_thread_ptr_ = 
	boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Record::ros_spin, this)));
	
	while(ros::ok() && !is_gps_ok_)
		usleep(10000);
		
	while(ros::ok())
	{
		if(keyEvent.keyMonitor(KeyEvent::KEY_Enter, KeyEvent::KEY_Up))
			this->recordToFile();
		usleep(30000);
	}
}

float Record::calculate_dis2(location_t & point1,location_t& point2)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude * M_PI/180.);
	float y = (point1.latitude - point2.latitude) *111000;
	return x*x+y*y;
}


void Record::gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
	if(system_delay_ >0)
	{
		system_delay_ --;
		return;
	}
	is_gps_ok_ = true;
	
	current_point.latitude = gps->latitude;
	current_point.longitude = gps->longitude;
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_gps_data_node");
	
	Record record;
	
	if(!record.init())
		return 0;

	record.run();
	
	return 0;
}


