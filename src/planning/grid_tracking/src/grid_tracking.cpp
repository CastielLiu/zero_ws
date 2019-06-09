#include"grid_tracking.h"


GridTracking::GridTracking():
	is_trafficLightGreen_(true),
	traffic_dir_(DIR_INVALID),
	is_gps_ok(false)
{
	target_point_index_=0;
	
	target_vertex_index_.row = 0;
	target_vertex_index_.col = 0;
	
	last_vertex_index_.row = -1;
	last_vertex_index_.col = 0;
	
	end_vertex_index_.row = 3;
	end_vertex_index_.col =0;

	cmd_.set_speed =0.0;
	cmd_.set_steeringAngle=0.0;
}

GridTracking::~GridTracking(){}

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
	if(!loadPathVertexes(file_path_))
		return false;
	
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&GridTracking::rosSpinThread, this)));
	
	while(!is_gps_ok && ros::ok())
	{
		usleep(500000);
		ROS_INFO("wait for gps initial...");
	}
	if(!generateStartPathPoints(path_points_))
		return false;

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

bool GridTracking::generateStartPathPoints(std::vector<gpsMsg_t>& path_points)
{
	gpsMsg_t start_point = starting_point_;
	gpsMsg_t end_point = path_vertexes_[0][0];
	
	int points_cnt = disBetween2Points(start_point,end_point)/DIS_INCREMENT;
	ROS_INFO("points_cnt:%d",points_cnt);
	
	if(points_cnt < 0)
	{
		ROS_ERROR("generateStartPathPoints failed");
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

bool GridTracking::generateEndPathPoints(std::vector<gpsMsg_t>& path_points)
{
	gpsMsg_t start_point = path_vertexes_[ROWS-1][0];
	gpsMsg_t end_point = ending_point_;
	
	int points_cnt = disBetween2Points(start_point,end_point)/DIS_INCREMENT;
	ROS_INFO("points_cnt:%d",points_cnt);
	
	if(points_cnt < 0)
	{
		ROS_ERROR("generateEndPathPoints failed");
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
	
	ROS_INFO("current_dir:%d",current_dir);
	
	if(!updateTargetVertexIndex(current_dir, traffic_dir, target_vertex_index_))
	{
		ROS_INFO("updateTargetVertexIndex failed");
		return;
	}

	if(target_vertex_index_.row >= ROWS || target_vertex_index_.col >= COLS)
	{
		ROS_INFO("target_vertex_index_ error");
		return;
	}
	
	gpsMsg_t end_point = path_vertexes_[target_vertex_index_.row][target_vertex_index_.col]; 
			
	size_t points_cnt = disBetween2Points(start_point,end_point)/DIS_INCREMENT;
	
	ROS_INFO("start_point: %f\t%f\t end_point: %f\t%f\t points_cnt:%d",
			start_point.longitude,start_point.latitude,end_point.longitude,end_point.latitude,points_cnt);
	
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
		if(target_point_index_ > path_points_.size()-1)
			break;
		else if(target_point_index_ > path_points_.size()-3.0/DIS_INCREMENT && //3m
				!target_vertex_index_.equal_to(end_vertex_index_))
		{
			cmd_.set_speed = 0.0;
			ROS_INFO("target_point_index_:%d\t path_points_size:%d",target_point_index_,path_points_.size());
			ROS_INFO("need to generateNewPathPoints");
			generateNewPathPoints(traffic_dir_, path_points_);
		}
		else if(target_vertex_index_.equal_to(end_vertex_index_))
			generateEndPathPoints(path_points_);
			
		target_point_ = path_points_[target_point_index_];
		std::pair<float, float> dis_yaw = getDisAndYaw(current_point_,target_point_,CONSIDER_DIR);
		
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
		
		if(i%100==0)
		{
			/*ROS_INFO("%.7f,%.7f,%.2f\t%.7f,%.7f\t t_yaw:%f\n",
					current_point_.longitude,current_point_.latitude,current_point_.yaw,
					target_point_.longitude,target_point_.latitude,dis_yaw.second);
			ROS_INFO("dis:%f\tyaw_err:%f\t Radius:%f\t t_roadWheelAngle:%f\n",
					dis_yaw.first,yaw_err,turning_radius,t_roadWheelAngle);*/
			ROS_INFO("target_index:%d\t size:%d\t traffic_dir:%d",target_point_index_, path_points_.size(),traffic_dir_);
		} i++;	
	}
	ROS_INFO("grid_tracking_node complete...");
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
}

bool GridTracking::loadPathVertexes(const std::string& file_path)
{
	FILE *fp = fopen(file_path.c_str(),"r");
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return false;
	}
	
	if(2 != fscanf(fp,"%lf\t%lf\n",&starting_point_.longitude,&starting_point_.latitude))
		return false;
	
	for(int i=0; i<ROWS*COLS; i++)
	{
		gpsMsg_t point;
		if(2 != fscanf(fp,"%lf\t%lf\n",&point.longitude,&point.latitude))
			return false;
		path_vertexes_[0][i] = point;
	}
	
	if(2 !=fscanf(fp,"%lf\t%lf\n",&ending_point_.longitude,&ending_point_.latitude))
		return false;

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
