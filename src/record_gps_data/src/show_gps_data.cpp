#include<ros/ros.h>
#include "gps_msgs/Inspvax.h"
#include <unistd.h>
#include<cmath>

#ifndef PI_
#define PI_ 3.141592653589
#endif


class Show
{
	private:
		void gps_callback(const gps_msgs::Inspvax::ConstPtr& gpsMsg);
		ros::Subscriber gps_sub;
		
	public:
		Show();
		~Show();
		bool init();
};

Show::Show()
{
}

Show::~Show()
{}

bool Show::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	gps_sub= nh.subscribe("/gps",1,&Show::gps_callback,this);
    return true;
}

void Show::gps_callback(const gps_msgs::Inspvax::ConstPtr & msg)
{
	ROS_INFO("%.7f\t%.7f\t%.2f\t%.2f\t%.2f",msg->longitude,msg->latitude,msg->north_velocity,msg->east_velocity,msg->up_velocity);
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"Show");
	
	Show show;
	
	if(!show.init())
		return 1;

	ros::spin();
	
	return 0;
}


