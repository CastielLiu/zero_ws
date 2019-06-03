#include"grid_tracking.h"


GridTracking::GridTracking():
	path_vertexes_ptr_(NULL)
{
	target_point_index_=0;
	is_gps_ok = false;

	cmd_.set_speed =0.0;

	cmd_.set_steeringAngle=0.0;
}

GridTracking::~GridTracking()
{
	if(path_vertexes_ptr_!= NULL)
		delet
}

bool GridTracking::load_path_vertexes(const std::string& file_path, gpsMsg_t path_vertexes[][])
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
		path_vertexes.push_back(point);
	}
	fclose(fp);
	return true;
}

bool GridTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	
	sub_gps_ = nh.subscribe("/gps",1,&GridTracking::gps_callback,this);
	timer_ = nh.createTimer(ros::Duration(0.01),&GridTracking::pub_gps_cmd_callback,this);
	
	pub_cmd_ = nh.advertise<driverless_msgs::ControlCmd>("/cmd",1);
	
	nh_private.param<float>("disThreshold",disThreshold_, 1.0);
	nh_private.param<float>("speed",speed_,1.0);
	nh_private.param<std::string>("file_path",file_path_,"");
	
	if(file_path_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	if(!load_path_vertexes(file_path_, path_vertexes))
		return false;
	
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::rosSpinThread, this)));
	
	
	float last_distance = FLT_MAX;
	float current_distance = 0;
	
	while(!is_gps_ok && ros::ok()) usleep(1000);

	ROS_INFO("path_points_.size:%d",path_points_.size());
	for(target_point_index_ =0; target_point_index_<path_points_.size(); )
	{
		
		target_point_ = path_points_[target_point_index_];
        std::pair<float, float> dis_yaw = get_dis_yaw(current_point_,target_point_);
		
		current_distance = get_dis_yaw(current_point_,target_point_).first;
		
		ROS_INFO("current_distance:%f\t last_distance:%f",current_distance,last_distance);
		printf("cur: %f\t%f\t tar: %f\t%f\r\n",current_point_.longitude,current_point_.latitude,target_point_.longitude,target_point_.latitude);
		if(current_distance - last_distance > 0)
		{
            target_point_index_--;
            target_point_ = path_points_[target_point_index_];
			break;
		}
		last_distance = current_distance;
		
		target_point_index_++;
	}
	
	if(target_point_index_ == path_points_.size())
	{
		ROS_ERROR("file read over, No target was found");
		return false;
	}
    return true;
}
void GridTracking::rosSpinThread()
{
	ros::spin();
}

void GridTracking::run()
{
	size_t i =0;
	while(ros::ok() && target_point_index_ < path_points_.size()-1)
	{
		if(current_point_.longitude <1.0 && current_point_.latitude <1.0)//初始状态或者数据异常
			continue;
		target_point_ = path_points_[target_point_index_];
		std::pair<float, float> dis_yaw = get_dis_yaw(current_point_,target_point_);
		
		if(target_point_index_ > path_points_.size()-10)
			cmd_.set_speed = 0.0;
		else
			cmd_.set_speed = speed_;
		
		if( dis_yaw.first < disThreshold_ || dis_yaw.first>1000.0)//初始状态下 target(0,0)-> dis_yaw.first 将会很大
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
		
		cmd_.set_steeringAngle = t_roadWheelAngle;
                
 		usleep(8000);
i++;
	}
}



void GridTracking::pub_cmd_callback(const ros::TimerEvent&)
{
		pub_cmd_.publish(cmd_);
}

void GridTracking::gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{
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
	
	GridTracking grid_tracking;
	if(!grid_tracking.init(nh,nh_private))
		return 1;
	
	grid_tracking.run();

	return 0;
}
