#include"reserve.h"


GridTracking::GridTracking():
	is_trafficLightGreen_(true),
	is_gps_ok(false)
{
	target_point_index_=0;
	
	cmd_.set_speed =0.0;
	cmd_.set_steeringAngle=0.0;
}

GridTracking::~GridTracking(){}

bool GridTracking::generatePathPoints(std::vector<gpsMsg_t>& path_points)
{
	int indexes[]={0,1,8,3,4,5,8,7,0,1,2,3,4,5,8,7};
	
	for(int i=0; i<sizeof(indexes)/sizeof(int)-1; i++)
	{
		gpsMsg_t start_point = path_vertexes_[indexes[i]];
		gpsMsg_t end_point = path_vertexes_[indexes[i+1]];
		
		int points_cnt = disBetween2Points(start_point,end_point)/DIS_INCREMENT;
		ROS_INFO("i:%d\t points_cnt:%d",i,points_cnt);
	
		double longitude_increment = (end_point.longitude - start_point.longitude)/points_cnt;
		double latitude_increment  = (end_point.latitude - start_point.latitude)/points_cnt;
		for(size_t j=0; j<points_cnt; j++)
		{
			gpsMsg_t point;
			point.longitude = start_point.longitude + longitude_increment *j;
			point.latitude = start_point.latitude + latitude_increment *j;
			path_points.push_back(point);
		}
	}
	return true;
}

bool GridTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_gps_ = nh.subscribe("/gps",1, &GridTracking::gps_callback,this);
	sub_traffic_sign_ = nh.subscribe("/traffic_sign",1, &GridTracking::traffic_sign_callback, this);
	timer_ = nh.createTimer(ros::Duration(0.04),&GridTracking::cmd_timer,this);
	sub_dump_ = nh.subscribe("/dump",0,&GridTracking::dump_callback,this);
	
	pub_cmd_ = nh.advertise<driverless_msgs::ControlCmd>("/cmd",1);
	
	nh_private.param<float>("disThreshold",disThreshold_, 1.0);
	nh_private.param<float>("tracking_speed",tracking_speed_,1.0);
	nh_private.param<std::string>("file_path",file_path_,"");
	
	if(file_path_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	if(!loadPathVertexes(file_path_, path_vertexes_))
		return false;
	
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&GridTracking::rosSpinThread, this)));
	
	while(!is_gps_ok && ros::ok())
	{
		usleep(500000);
		ROS_INFO("wait for gps initial...");
	}
	
	if(!generatePathPoints(path_points_))
		return false;
	
	std::string dump_file_path = file_path_.substr(0,file_path_.find_last_of("/")+1) + "raw_path.txt";
	//ROS_ERROR("%s",dump_file_path.c_str());
	dumpPathPoints(dump_file_path,path_points_);

	for(target_point_index_ =0; target_point_index_<path_points_.size(); )
	{
		target_point_ = path_points_[target_point_index_];
		
		float current_distance = disBetween2Points(current_point_,target_point_,CONSIDER_DIR);
		
		ROS_INFO("current_distance:%f",current_distance);
		printf("cur: %f\t%f\t tar: %f\t%f\r\n",
				current_point_.longitude,current_point_.latitude,target_point_.longitude,target_point_.latitude);
		if(current_distance > 0)
			break;
		
		target_point_index_++;
	}
	if(target_point_index_ == path_points_.size())
	{
		ROS_ERROR("No target point was found");
		return false;
	}
	ROS_INFO("grid tracking node initial ok. target index: %d",target_point_index_);
    return true;
}

void GridTracking::run()
{
	size_t i =0;
	while(ros::ok())
	{
		if(target_point_index_ > path_points_.size()-1)
			break;
			
		target_point_ = path_points_[target_point_index_];
		
		std::pair<float, float> dis_yaw = getDisAndYaw(current_point_,target_point_);
		
		if( dis_yaw.first < disThreshold_)
		{
			target_point_index_ ++;
			continue;
		}
			
		float yaw_err = dis_yaw.second - current_point_.yaw;

		if(yaw_err==0.0) continue;
		
		float turning_radius = (0.5 * dis_yaw.first)/sin(deg2rad(yaw_err));
		
		float t_roadWheelAngle = -generateRoadwheelAngleByRadius(turning_radius);
		
		saturate_cast<float>(t_roadWheelAngle,25.0);
		
		if(is_trafficLightGreen_)
			cmd_.set_speed = tracking_speed_;
		else
			cmd_.set_speed = 0.0;
		cmd_.set_steeringAngle = t_roadWheelAngle;
                
 		usleep(10000);
		
		if(i%100==0)
		{
			ROS_INFO("%.7f,%.7f,%.2f\t%.7f,%.7f\t t_yaw:%f\n",
					current_point_.longitude,current_point_.latitude,current_point_.yaw,
					target_point_.longitude,target_point_.latitude,dis_yaw.second);
			ROS_INFO("dis:%f\tyaw_err:%f\t Radius:%f\t t_roadWheelAngle:%f\n",
					dis_yaw.first,yaw_err,turning_radius,t_roadWheelAngle);
		} i++;	
	}
	
	ROS_INFO("grid_tracking_node complete...");
	
	while(ros::ok())
	{
		cmd_.set_speed = 0.0;
		cmd_.set_steeringAngle = 0.0;
		usleep(100000);
	}
	
	
}


void GridTracking::cmd_timer(const ros::TimerEvent&)
{
	pub_cmd_.publish(cmd_);
}

void GridTracking::gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
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

void GridTracking::traffic_sign_callback(const driverless_msgs::TrafficSign::ConstPtr &msg)
{
	if(msg->traffic_light_validity && msg->traffic_light != driverless_msgs::TrafficSign::GreenLight)
		is_trafficLightGreen_ = false;
	else
		is_trafficLightGreen_ = true;
}

bool GridTracking::loadPathVertexes(const std::string& file_path, std::vector<gpsMsg_t>& path_vertexes)
{
	FILE *fp = fopen(file_path.c_str(),"r");
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return false;
	}
	
	while(!feof(fp))
	{
		gpsMsg_t point;
		fscanf(fp,"%lf\t%lf\n",&point.longitude,&point.latitude);
		path_vertexes.push_back(point);
	}

	fclose(fp);
	
	return true;
}

bool GridTracking::dumpPathPoints(const std::string& file_path, const std::vector<gpsMsg_t>& path_points)
{
	FILE *fp = fopen(file_path.c_str(),"w");
	
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return false;
	}
	
	for(size_t i=0;i<path_points.size();i++)
		fprintf(fp,"%lf\t%lf\n",path_points[i].longitude, path_points[i].latitude);
			
	fclose(fp);
	
	return true;
}


void GridTracking::dump_callback(const std_msgs::String::ConstPtr& msg)
{
	dumpPathPoints(msg->data,path_points_);
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
