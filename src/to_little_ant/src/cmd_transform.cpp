#include<iostream>
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd1.h>
#include<little_ant_msgs/ControlCmd2.h>
#include<driverless_msgs/ControlCmd.h>
#include<driverless_utils/vehicle_params.h>

class Transform
{
public:
	Transform()
	{
		little_ant_cmd1.set_driverlessMode = true;
		little_ant_cmd2.set_gear = 1;
		little_ant_cmd2.set_speed = 0.0;
		little_ant_cmd2.set_brake =0.0;
		little_ant_cmd2.set_steeringAngle = 0.0;
	}
	~Transform(){}
	
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
		sub_cmd = nh.subscribe("/cmd",0,&Transform::cmd_callback,this);
	
		timer_10ms = nh.createTimer(ros::Duration(0.01),&Transform::timer_10ms_callback,this);
		pub_cmd1 = nh.advertise<little_ant_msgs::ControlCmd1>("controlCmd1",0);
		pub_cmd2 = nh.advertise<little_ant_msgs::ControlCmd2>("controlCmd2",0);
	}
	
	void cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& cmd)
	{
		little_ant_cmd2.set_steeringAngle = cmd->set_steeringAngle *g_steering_gearRatio;
		little_ant_cmd2.set_speed = cmd->set_speed;
	}
	
	void timer_10ms_callback(const ros::TimerEvent&)
	{
		static int i=0;
		pub_cmd2.publish(little_ant_cmd2);
		if(i%2==0)
			pub_cmd1.publish(little_ant_cmd1);
		i++;
	}

private:
	little_ant_msgs::ControlCmd1 little_ant_cmd1;
	little_ant_msgs::ControlCmd2 little_ant_cmd2;
	ros::Subscriber sub_cmd;
	ros::Timer timer_10ms;
	ros::Publisher pub_cmd1;
	ros::Publisher pub_cmd2;
	
};


int main(int argc,char** argv)
{
	ros::init(argc,argv,"cmd_transform_node");
	Transform transform;
	transform.init();
	ros::spin();
}

