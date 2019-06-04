#include"grid_tracking.h"


GridTracking::GridTracking():
	is_trafficLightGreen_(true),
	traffic_dir_(driverless_msgs::TrafficSign::DirectDir),
	is_gps_ok(false)
{
	target_point_index_=0;
	target_vertex_index_.row = 0;
	target_vertex_index_.col = 0;
	
	last_vertex_index_.row = 0;
	last_vertex_index_.col = 0;

	cmd_.set_speed =0.0;
	cmd_.set_steeringAngle=0.0;
}

GridTracking::~GridTracking(){}

bool GridTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_gps_ = nh.subscribe("/gps",1, &GridTracking::gps_callback,this);
	sub_traffic_sign_ = nh.subscribe("/traffic_sign",1, &GridTracking::traffic_sign_callback, this);
	timer_ = nh.createTimer(ros::Duration(0.04),&GridTracking::cmd_timer,this);
	
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
	if(!generateOriginPathPoints(path_points_))
		return false;

	for(target_point_index_ =0; target_point_index_<path_points_.size(); )
	{
		target_point_ = path_points_[target_point_index_];
		
		float current_distance = disBetween2Points(current_point_,target_point_);
		
		ROS_INFO("current_distance:%f",current_distance);
		printf("cur: %f\t%f\t tar: %f\t%f\r\n",
				current_point_.longitude,current_point_.latitude,target_point_.longitude,target_point_.latitude);
		if(current_distance > 0)
		{
            target_point_ = path_points_[--target_point_index_];
			break;
		}
		target_point_index_++;
	}
	if(target_point_index_ == path_points_.size())
	{
		ROS_ERROR("No target point was found");
		return false;
	}
    return true;
}

bool GridTracking::generateOriginPathPoints(std::vector<gpsMsg_t>& path_points)
{
	gpsMsg_t start_point = current_point_;
	gpsMsg_t end_point = path_vertexes_[0][0];
	
	int points_cnt = disBetween2Points(start_point,end_point)/DIS_INCREMENT;
	ROS_INFO("points_cnt:%d",points_cnt);
	
	if(points_cnt < 0)
	{
		ROS_ERROR("generateOriginPathPoints failed");
		return false;
	}
	
	double longitude_increment = (end_point.longitude - start_point.longitude)/points_cnt;
	double latitude_increment  = (end_point.latitude - start_point.latitude)/points_cnt;
	for(size_t i=0; i<points_cnt; i++)
	{
		gpsMsg_t point;
		point.longitude = start_point.longitude + longitude_increment *i;
		point.latitude = start_point.latitude + latitude_increment *i;
		path_points.push_back(point);
	}
	return true;
}

uint8_t GridTracking::generateCurrentDir(const vertexIndex_t& target_index, const vertexIndex_t& last_index)
{
	uint8_t row_diff = target_index.row-last_index.row;
	uint8_t col_diff = target_index.col-last_index.col;
	//relative to the world coordinate
	//3 up
	//1 down
	//0 left
	//4 right
	return row_diff+2*col_diff+2;
}

bool GridTracking::updateTargetVertexIndex(uint8_t current_dir, uint8_t traffic_dir, vertexIndex_t& target_vertex_index)
{
	if(traffic_dir == DIR_INVALID) //no traffic sign or no sign has been detected yet
	{
		if(target_vertex_index.row == 0 && target_vertex_index.col==COLS-1) //lower-right
		{
			if(current_dir==4)//right
				target_vertex_index.row++;
			else //down
				target_vertex_index.col--;
		}
		else if(target_vertex_index.row == ROWS-1 && target_vertex_index.col==COLS-1) //upper-right
		{
			if(current_dir==3) //up
				target_vertex_index.col--;
			else //right
				target_vertex_index.row--;
		}
		else //upper-left  pass  //lower-left  pass
			return false;
		return true;
	}

	uint8_t result = current_dir*3 + traffic_dir;
	if(result==2 || result==9 || result== 13) //row++
		target_vertex_index.row++;
	else if(result==1 || result==3 ||result==14) //row--
		target_vertex_index.row--;
	else if(result==4 || result==11 || result==12) //col++
		target_vertex_index.col ++;
	else if(result==0 || result==5 ||result==10) //col--
		target_vertex_index.col--;
	return true;
}

void GridTracking::generateNewPathPoints(uint8_t& traffic_dir, std::vector<gpsMsg_t>& path_points)
{
	vertexIndex_t temp = target_vertex_index_;
	
	gpsMsg_t start_point = path_vertexes_[target_vertex_index_.row][target_vertex_index_.col];
	
	uint8_t current_dir = generateCurrentDir(target_vertex_index_,last_vertex_index_);
	
	if(!updateTargetVertexIndex(current_dir, traffic_dir, target_vertex_index_))
		return;
		
	if(target_vertex_index_.row >= ROWS || target_vertex_index_.col >= COLS)
		return;
	
	gpsMsg_t end_point = path_vertexes_[target_vertex_index_.row][target_vertex_index_.col]; 
			
	size_t points_cnt = disBetween2Points(start_point,end_point)/DIS_INCREMENT;
	
	double longitude_increment = (end_point.longitude - start_point.longitude)/points_cnt;
	double latitude_increment  = (end_point.latitude - start_point.latitude)/points_cnt;
	for(size_t i=0; i<points_cnt; i++)
	{
		gpsMsg_t point;
		point.longitude = start_point.longitude + longitude_increment *i;
		point.latitude = start_point.latitude + latitude_increment *i;
		path_points.push_back(point);
	}
	last_vertex_index_ = temp;
}

void GridTracking::run()
{
	size_t i =0;
	while(ros::ok())
	{
		if(target_point_index_ > path_points_.size()-3.0/DIS_INCREMENT) //3m
			generateNewPathPoints(traffic_dir_, path_points_);
		else if(target_point_index_ > path_points_.size()-1)
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
		
		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		saturate_cast<float>(t_roadWheelAngle,25.0);
		
		if(is_trafficLightGreen_)
			cmd_.set_speed = tracking_speed_;
		else
			cmd_.set_speed = 0.0;
		cmd_.set_steeringAngle = t_roadWheelAngle;
                
 		usleep(10000);
		
		if(i%50==0)
		{
			ROS_INFO("%.7f,%.7f,%.2f\t%.7f,%.7f\t t_yaw:%f\n",
					current_point_.longitude,current_point_.latitude,current_point_.yaw,
					target_point_.longitude,target_point_.latitude,dis_yaw.second);
			ROS_INFO("dis:%f\tyaw_err:%f\t Radius:%f\t t_roadWheelAngle:%f\n",
					dis_yaw.first,yaw_err,turning_radius,t_roadWheelAngle);
		} i++;	
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
	
	if(msg->direction_validity)
		traffic_dir_ = msg->direction;
	else
		traffic_dir_ = DIR_INVALID;
}

bool GridTracking::loadPathVertexes(const std::string& file_path,  gpsMsg_t path_vertexes[ROWS][COLS])
{
	FILE *fp = fopen(file_path.c_str(),"r");
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return false;
	}
	gpsMsg_t point;
	int i=0;
	
	while(!feof(fp) && i< ROWS*COLS)
	{
		fscanf(fp,"%lf\t%lf\t",&point.longitude,&point.latitude);
		path_vertexes[0][i] = point;
		i++;
	}
	fclose(fp);
	
	if(i != ROWS*COLS)
	{
		ROS_ERROR("path vertex count error!!!");
		return false;
	}
	return true;
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
