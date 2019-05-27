#ifndef MSG_HANDLER_H_
#define MSG_HANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include<serial/serial.h>
#include<arpa/inet.h>

#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<driverless_msgs/ControlCmd.h>

#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

//for stm32
#define STM32_MAX_LOAD_SIZE 100

#define STM32_MAX_PKG_BUF_LEN 20

uint8_t stm32_data_buf[STM32_MAX_LOAD_SIZE];
uint8_t stm32_pkg_buf[STM32_MAX_PKG_BUF_LEN];

enum{Stm32MsgHeaderByte0=0x66,Stm32MsgHeaderByte1=0xCC};

PACK(
typedef struct 
{
	uint8_t header0;
	uint8_t header1;
	uint8_t pkgLen;
	uint8_t id;
	uint16_t speed;
	uint16_t steeringAngle;
	uint8_t reserved;
	uint8_t checkNum;
	
}) writePkg_t;

PACK(
typedef struct 
{
	uint8_t header0;
	uint8_t header1;
	uint8_t pkgLen;
	uint8_t id;
	uint16_t speed;
	uint16_t steeringAngle;
	uint8_t reserved;
	uint8_t checkNum;
	
}) readPkg_t;

// end for stm32


class BaseControl
{
public:
	BaseControl();
	~BaseControl();
	bool init(int ,char**);
	void run();
	
	void read_stm32_port();

	void cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& cmd);
	
private:
	void Stm32BufferIncomingData(unsigned char *message, unsigned int length);
	void parse_stm32_msgs(unsigned char *msg);
	uint8_t generateCheckNum(const void* ,size_t len);
	
private:
	serial::Serial * stm32_serial_port_;
	
	writePkg_t write_msg_;
	
	boost::shared_ptr<boost::thread> readFromStm32_thread_ptr_; //智能指针 
	
	ros::Subscriber sub_cmd_;
	
	std::string stm32_port_name_;
	int stm32_baudrate_;
	
	float max_steering_speed_;  //Front and rear frame maximun steering angle difference 
	
	boost::mutex mutex_;

};


#endif
