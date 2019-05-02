#include"nuogeng/nuogeng.h"



Nuogeng::Nuogeng():
	data_index_(0),
	serial_(NULL)
{
	buffer_ = new unsigned char[MAX_NOUT_SIZE];
	data_buffer_ = new unsigned char[MAX_PKG_LEN];
	gpsPtr = (NuogengGps_t *)data_buffer_;
	
	memset(&inspax,0,sizeof(inspax));
}

Nuogeng::~Nuogeng()
{
	if(serial_->isOpen())
		serial_->close();
		
	if(serial_!=NULL)
	{
		delete serial_;
		serial_ = NULL;
	}
	delete [] buffer_;
	delete [] data_buffer_;
	ROS_INFO("nuogeng_node is shutdown");
}

bool Nuogeng::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	pub_gps_ = nh.advertise<gps_msgs::Inspvax>("/gps",1);
	
	nh_private.param<std::string>("port_name",port_name_,"/dev/ttyUSB0");
	nh_private.param<int>("baudrate",baudrate_,115200);
	
	try
	{
		serial_ = new Serial(port_name_,baudrate_,Timeout::simpleTimeout(10));
		if(!serial_->isOpen())
		{
			 std::stringstream output;
	        output << "Serial port: " << port_name_ << " failed to open." << std::endl;
			delete serial_;
			serial_ = NULL;
			return false;
		}
		else
		{
			std::stringstream output;
			output << "Serial port: " << port_name_ << " opened successfully." << std::endl;
			std::cout << output.str() <<std::endl;
		}
		serial_->flush();
	}
	catch(std::exception &e)
	{
		std::stringstream output;
	    output << "Error  " << port_name_ << ": " << e.what();
	    std::cout << output.str() <<std::endl;
		return false;
	}
	ROS_INFO("nuo geng gps initial ok .");
	return true;
	
}

void Nuogeng::run()
{
	int len = 0;
	ros::Rate loop_rate(30);
	while(ros::ok())
	{
		try
		{
			len = serial_->read(buffer_, MAX_NOUT_SIZE);
			//ROS_INFO("len:%d",len);
		} 
		catch (std::exception &e) 
		{
	        std::stringstream output;
	        output << "Error reading from serial port: " << e.what();
	        std::cout << output.str() <<std::endl;
    	}
		if(len==0) continue;
		
		BufferIncomingData(buffer_,len);
		loop_rate.sleep();
		
	}
}

void Nuogeng::BufferIncomingData(unsigned char *message,size_t len)
{
	for(size_t i=0;i<len;i++)
	{
		switch(data_index_)
		{
			case 1:
				if(message[i] == 0x14)
					data_buffer_[data_index_++] = message[i];
				else
					data_index_ = 0;
					break;
				
			case 2:
				if(message[i] == 0x64)
					data_buffer_[data_index_++] = message[i];
				else
					data_index_ =0;
					break;
				
			case 4:
				data_buffer_[data_index_++] = message[i];
				if(data_buffer_[0] != generateLRC(data_buffer_+1,4))
					data_index_ = 0;
					break;
				
			case 104:
				data_buffer_[data_index_++] = message[i];
				if(*(uint16_t*)(data_buffer_+3) == generateCRC(data_buffer_+5,100))
					this->parseMessage();
				data_index_ = 0;
				break;
			
			default :
				data_buffer_[data_index_++] = message[i];
				break;
		}
	}
}

void Nuogeng::parseMessage()
{
	inspax.header.stamp = ros::Time::now();
	inspax.header.frame_id = "gps";
	inspax.latitude = gpsPtr->latitude;
	inspax.longitude = gpsPtr->longitude;
	inspax.height = gpsPtr->height;
	inspax.north_velocity = gpsPtr->north_velocity;
	inspax.east_velocity = gpsPtr->east_velocity;
	inspax.up_velocity = gpsPtr->down_velocity;
	inspax.roll = gpsPtr->roll;
	inspax.pitch = gpsPtr->pitch;
	inspax.azimuth = gpsPtr->yaw;
	inspax.latitude_standard_deviation = gpsPtr->latitude_std_deviation;
	inspax.longitude_standard_deviation = gpsPtr->longitude_std_deviation;
	inspax.height_standard_deviation = gpsPtr->height_std_deviation;
	
	pub_gps_.publish(inspax);
	 
}

uint8_t Nuogeng::generateLRC(uint8_t* buf,size_t len)
{
	uint8_t sum = 0;
	for(size_t i=0; i<len; i++)
	{
		sum+=buf[i];
	}
	return 0x100 - sum;
}


verify_width_t Nuogeng::generateCRC(unsigned char *buf,unsigned int nBytes)
{
	verify_width_t remainder = INITIAL_REMAINDER;
	unsigned int byte;
	for(int i=0;i<nBytes;i++)
	{
		byte = (remainder >> (8*sizeof(verify_width_t)-8) ^ buf[i] );
		remainder = crcTable[byte] ^ (remainder <<8);	
	}
	return remainder;
}

void Nuogeng::crcTableGenerate(unsigned char *crc_table)
{
 	verify_width_t remainder;  
    verify_width_t i; 
    for(i=0;i<256;i++)
    {
    	remainder = i << (8*sizeof(verify_width_t)-8);
    	for(unsigned char bit=0;bit<8;bit++)
    	{
    		if(remainder & 0x8000) 
    			remainder = (remainder << 1) ^ CRC16_CCITT;
    		else
    			remainder <<=1;
		}
		crc_table[i] = remainder;
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"nuogeng_node");
	Nuogeng nuogeng;
	if(nuogeng.init())
		nuogeng.run();
	return 0;
}
