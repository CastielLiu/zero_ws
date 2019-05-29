#include<base_control/base_control.h>
#include<assert.h>

static bool openSerial(serial::Serial* & port_ptr, std::string port_name,int baud_rate)
{
	try 
	{
		port_ptr = new serial::Serial(port_name,baud_rate,serial::Timeout::simpleTimeout(10)); 

		if (!port_ptr->isOpen())
		{
	        std::stringstream output;
	        output << "Serial port: " << port_name << " failed to open." << std::endl;
			delete port_ptr;
			port_ptr = NULL;
			return false;
		} 
		else 
		{
	        std::stringstream output;
	        output << "Serial port: " << port_name << " opened successfully." << std::endl;
	        std::cout << output.str() <<std::endl;
		}

		port_ptr->flush();
	} 
	catch (std::exception &e) 
	{
	    std::stringstream output;
	    output << "Error  " << port_name << ": " << e.what();
	    std::cout << output.str() <<std::endl;
	    return false;
	}
	
	return true;

}

BaseControl::BaseControl()
{
	stm32_serial_port_ = NULL;

	write_msg_.header0 = Stm32MsgHeaderByte0;
	write_msg_.header1 = Stm32MsgHeaderByte1;
	write_msg_.reserved = 0x00;

}

BaseControl::~BaseControl()
{
	stm32_serial_port_->close();
	if(stm32_serial_port_!=NULL)
	{
		delete stm32_serial_port_;
		stm32_serial_port_ = NULL;
	}
}

bool BaseControl::init(int argc,char**argv)
{
	ros::init(argc,argv,"base_control");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<std::string>("stm32_port_name", stm32_port_name_, "/dev/ttyUSB0");
	nh_private.param<float>("max_steering_speed",max_steering_speed_,5.0);
	nh_private.param<int>("stm32_baudrate",stm32_baudrate_,115200);
	
	
	assert(max_steering_speed_>0);
	
	sub_cmd_ = nh.subscribe("/cmd",1,&BaseControl::cmd_callback,this);
	
	if(!openSerial(stm32_serial_port_,stm32_port_name_,stm32_baudrate_))
		return false;
	
	ROS_INFO("System initialization completed");
	
	return true;
}

void BaseControl::run()
{
	readFromStm32_thread_ptr_ = boost::shared_ptr<boost::thread > 
		(new boost::thread(boost::bind(&BaseControl::read_stm32_port, this)));
		
	ros::spin();
}

void BaseControl::read_stm32_port()
{
	size_t len;
	
	stm32_serial_port_->flushInput();
	
	while(ros::ok())
	{
		//ROS_INFO("read_stm32_port  ing.....");
		usleep(5000);
		try 
		{
			len = stm32_serial_port_->read(stm32_data_buf, STM32_MAX_LOAD_SIZE);
			//ROS_INFO("read_stm32_port get %d bytes",len);
		}
		catch (std::exception &e) 
		{
			std::stringstream output;
			output << "Error reading from serial port: " << e.what();
			std::cout << output.str() <<std::endl;
		}
		if(len == 0) continue;

		/*for(int i=0;i<len;i++)
			printf("%x\t",stm32_data_buf[i]);
		std::cout << len << std::endl;*/
		
		Stm32BufferIncomingData(stm32_data_buf, len);
	}
}

void BaseControl::Stm32BufferIncomingData(unsigned char *message, unsigned int length)
{
	if(message[0] == 1)
		ROS_INFO("STM32 heared command...");
	else
		ROS_INFO("STM32 not heared command...");
/*
	static int buffer_index = 0;
	static int bytes_remaining =0;
	
	// add incoming data to buffer
	for (unsigned int ii=0; ii<length; ii++) 
	{// make sure bufIndex is not larger than buffer
		if(buffer_index >= STM32_MAX_PKG_BUF_LEN)
		{
			buffer_index = 0;
			printf("Overflowed receive buffer. Buffer cleared.");
		}
		switch(buffer_index)
		{
			case 0: //nothing
				if(message[ii]==Stm32MsgHeaderByte0)
				{
					stm32_pkg_buf[buffer_index++] = message[ii];
				}
				bytes_remaining = 0;
				break;
			case 1:
				if(message[ii]==Stm32MsgHeaderByte1)
					stm32_pkg_buf[buffer_index++] = message[ii];
				else
				{
					buffer_index = 0;
					bytes_remaining = 0;
				}
				break;
			case 2:
				stm32_pkg_buf[buffer_index++]=message[ii];

				bytes_remaining = stm32_pkg_buf[2];
				//根据实际发送的包长最大小值进行限定(多重数据正确保障) 
				if(bytes_remaining > 10 || bytes_remaining < 2)  
					buffer_index = 0;
				break;
			default:
				stm32_pkg_buf[buffer_index++] = message[ii];
				bytes_remaining --;
				if(bytes_remaining == 0)
				{
					parse_stm32_msgs(stm32_pkg_buf);
					buffer_index = 0;
				}
				break;
		}
	}// end for
	*/
}

void BaseControl::parse_stm32_msgs(unsigned char *msg)
{
	unsigned char pkgId = msg[3];
	if(pkgId == 0x00)
	{
		readPkg_t *read_pkgPtr = (readPkg_t *)msg;
		if(read_pkgPtr->checkNum != generateCheckNum(read_pkgPtr,sizeof(readPkg_t)))
		{
			ROS_ERROR("check error");
			return ;
		}
		/*
		uint16_t speed = read_pkgPtr->speed;
		uint16_t steeringAngle = read_pkgPtr->steeringAngle;
		if(write_msg_.speed != speed || steeringAngle != write_msg_.steeringAngle)
		{
			ROS_ERROR("data error!!");
			printf("%d\t%d\t\t%d\t%d",write_msg_.speed,speed,steeringAngle,write_msg_.steeringAngle);
		}
		else
			ROS_INFO("right");
		*/
	}
}

void printCmdBytes(uint8_t *msg,int len)
{
	for(size_t i=0;i<len;i++)
		printf("%x\t",msg[i]);
	printf("\n");
}

void BaseControl::cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& cmd)
{
	write_msg_.pkgLen = sizeof(writePkg_t)-3;
	write_msg_.id = 0x01;
	write_msg_.speed = cmd->set_speed*100 + 65535/2;
	write_msg_.steeringAngle = cmd->set_steeringAngle*100 + 65535/2;
	write_msg_.checkNum = generateCheckNum(&write_msg_,sizeof(write_msg_));
	
	stm32_serial_port_->write((uint8_t *)&write_msg_,sizeof(write_msg_));
	
	//printf("%d\t%d\r\n",write_msg_.speed,write_msg_.steeringAngle);
	
	//printCmdBytes((uint8_t *)&write_msg_,sizeof(write_msg_));
}

uint8_t BaseControl::generateCheckNum(const void* buf,size_t len)
{
	const uint8_t * ptr = (const uint8_t *)buf;
	uint8_t sum=0;

	for(int i=2; i<len-1 ; i++)
	{
		sum += ptr[i];
	}
	return sum;
}


int main(int argc,char**argv)
{
	BaseControl base_control;
	
	if(base_control.init(argc,argv))
		base_control.run();
	
	ROS_INFO("base_control_node has exited");
	return 0;
}



