#include"path_tracking.h"


PathTracking::PathTracking()
{
	target_point_index_=0;
	is_gps_ok = false;
	

	gps_controlCmd_.set_speed =0.0;

	gps_controlCmd_.set_steeringAngle=0.0;
}

PathTracking::~PathTracking()
{

}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_gps_ = nh.subscribe("/gps",5,&PathTracking::gps_callback,this);
	//sub_vehicleState2_ = nh.subscribe("/vehicleState2",5,&PathTracking::vehicleSpeed_callback,this);
	//sub_avoiding_from_lidar_ = nh.subscribe("/start_avoiding",2,&PathTracking::avoiding_flag_callback,this);
	
	timer_ = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_gps_cmd_callback,this);
	
	pub_gps_cmd_ = nh.advertise<driverless_msgs::ControlCmd>("/cmd",5);
	
	nh_private.param<std::string>("path_points_file",path_points_file_,"");
	nh_private.param<float>("disThreshold",disThreshold_, 1.0);
	nh_private.param<float>("speed",speed_,1.0);
	
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}	

	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::rosSpinThread, this)));

    if(!load_path_points(path_points_file_, path_points_))
        return false;
	
	while(!is_gps_ok && ros::ok()) 
	{
		usleep(500000);
		ROS_INFO("wait for gps initial...");
	}

	ROS_INFO("path_points_.size:%d",path_points_.size());
	for(target_point_index_ =0; target_point_index_<path_points_.size(); )
	{
		target_point_ = path_points_[target_point_index_];
		
		float current_distance = disBetween2Points(current_point_,target_point_,CONSIDER_DIR);
		
		ROS_INFO("current_distance:%f",current_distance);

		printf("cur: %f\t%f\t tar: %f\t%f\r\n",
				current_point_.longitude,current_point_.latitude,target_point_.longitude,target_point_.latitude);
		if(current_distance > 0)
		{
            target_point_ = path_points_[target_point_index_];
			break;
		}
		
		target_point_index_++;
	}
	
	if(target_point_index_ == path_points_.size())
	{
		ROS_ERROR("file read over, No target was found");
		return false;
	}
    return true;
}
void PathTracking::rosSpinThread()
{
	ros::spin();
}

void PathTracking::run()
{
	size_t i =0;
	while(ros::ok() && target_point_index_ < path_points_.size()-1)
	{
		target_point_ = path_points_[target_point_index_];
		std::pair<float, float> dis_yaw = getDisAndYaw(current_point_,target_point_);
		
		if(target_point_index_ > path_points_.size()-10)
			gps_controlCmd_.set_speed = 0.0;
		else
			gps_controlCmd_.set_speed = speed_;
		
		if( dis_yaw.first < disThreshold_ || dis_yaw.first>50.0)
		{
			target_point_index_ ++;
			continue;
		}
			
		float yaw_err = dis_yaw.second - current_point_.yaw;
          //float yaw_err = 10.0;

		if(yaw_err==0.0) continue;
		
		float turning_radius = (0.5 * dis_yaw.first)/sin(deg2rad(yaw_err));
		
		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		
if(i%20==0){
		ROS_INFO("%.7f,%.7f,%.2f\t%.7f,%.7f\t t_yaw:%f\n",
				current_point_.longitude,current_point_.latitude,current_point_.yaw,
				target_point_.longitude,target_point_.latitude,dis_yaw.second);
		ROS_INFO("dis:%f\tyaw_err:%f\t Radius:%f\t t_roadWheelAngle:%f\n",dis_yaw.first,yaw_err,turning_radius,t_roadWheelAngle);
	}	
		saturate_cast<float>(t_roadWheelAngle,25.0);
		
		gps_controlCmd_.set_steeringAngle = t_roadWheelAngle;
                

 		usleep(8000);
i++;
	}
}

void PathTracking::pub_gps_cmd_callback(const ros::TimerEvent&)
{
	pub_gps_cmd_.publish(gps_controlCmd_);
}

void PathTracking::gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{
	static int system_delay = 10;
	if(system_delay)
	{
		system_delay--;
		return;
	}
	is_gps_ok = true;
	current_point_.longitude = msg->longitude;
	current_point_.latitude = msg->latitude;
	current_point_.yaw = msg->azimuth;
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,"path_tracking_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	PathTracking path_tracking;
	if(!path_tracking.init(nh,nh_private))
		return 1;
	
	path_tracking.run();

	return 0;
}
