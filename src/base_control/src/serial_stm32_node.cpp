#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 

#include "data_deal.h"
#include<driverless_msgs/ControlCmd.h>

#define sendnum 2                //上位机发送给stm32的数据容量
#define sendlen (sendnum+1)*4
#define recvnum 2               //stm32发送给上位机的数据容量
#define recvlen (recvnum+1)*4

serial::Serial ser; //声明串口对象
//定义接收，发送数据
float senddata[sendnum];
u8 sendbuff[sendlen];
float recvdata[recvnum];
u8 recvbuff[recvlen];


//发送回调函数 
void send_callback(const driverless_msgs::ControlCmd::ConstPtr& cmd) 
{ 
	//int i = 0;
	senddata[0] = cmd->set_speed;
	senddata[1] = cmd->set_steeringAngle;
    sendfloat(sendbuff,senddata,sendnum);
	ser.write(sendbuff,sendlen);   //发送串口数据
}


int main (int argc, char** argv) 
{ 
	int i = 0;
	for(i=0;i<sendnum;i++)
		senddata[i] = 0;	
	ros::init(argc, argv, "serial_stm32_node"); 
	ros::NodeHandle nh; 

	ros::Subscriber write_sub = nh.subscribe("/sensor_decision", 1, send_callback); 

	try 
	{ 
		//设置串口属性，并打开串口 
		ser.setPort("/dev/ttyUSB0"); 
		ser.setBaudrate(115200); 
		serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
		ser.setTimeout(to); 
		ser.open(); 
	 }
	catch (serial::IOException& e) 
	{ 
		ROS_ERROR_STREAM("Unable to open port "); 
		return -1; 
	} 

	//检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()) 
		ROS_INFO_STREAM("Serial Port initialized"); 
	else 
		return -1; 

	ros::Rate loop_rate(20); 
	while(ros::ok()) 
	{ 
		if(ser.available())
		{ 
			//ROS_INFO_STREAM("Reading from serial port");  
			//ser.read(recvbuff,recvsize);
		} 
		ros::spinOnce(); 
		loop_rate.sleep(); 
	} 
	return 0;
} 
