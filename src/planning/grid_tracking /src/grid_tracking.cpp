#include"grid_tracking.h"


GridTracking::GridTracking():
	traffic_light_(driverless_msgs::TrafficSign::GreenLight),
	traffic_dir_(driverless_msgs::TrafficSign::DirectDir),
	is_gps_ok(false)
{
	target_point_index_=0;
	target_vertex_index_.row = 0;
	target_vertex_index_.col = 0;
	
	last_vetex_index_.row = -1;
	last_vetex_index_.col = 0;

	cmd_.set_speed =0.0;
	cmd_.set_steeringAngle=0.0;
}

GridTracking::~GridTracking()
{

}

bool GridTracking::loadPathVertexes(const std::string& file_path,  gpsMsg_t** path_vertexes)
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

bool GridTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_gps_ = nh.subscribe("/gps",1,&GridTracking::gps_callback,this);
	timer_ = nh.createTimer(ros::Duration(0.04),&GridTracking::pub_gps_cmd_callback,this);
	
	pub_cmd_ = nh.advertise<driverless_msgs::ControlCmd>("/cmd",1);
	
	nh_private.param<float>("disThreshold",disThreshold_, 1.0);
	nh_private.param<float>("speed",speed_,1.0);
	nh_private.param<std::string>("file_path",file_path_,"");
	
	if(file_path_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	if(!load_path_vertexes(file_path_, path_vertexes_))
		return false;
	
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&GridTracking::rosSpinThread, this)));
	
	while(!is_gps_ok && ros::ok()) usleep(1000);

	generateOriginPathPoints(path_points_);
	
	float current_distance, last_distance = FLT_MAX;

	for(target_point_index_ =0; target_point_index_<path_points_.size(); )
	{
		target_point_ = path_points_[target_point_index_];
		
		current_distance = point2point_dis(current_point_,target_point_);
		
		ROS_INFO("current_distance:%f\t last_distance:%f",current_distance,last_distance);
		printf("cur: %f\t%f\t tar: %f\t%f\r\n",
				current_point_.longitude,current_point_.latitude,target_point_.longitude,target_point_.latitude);
		if(current_distance - last_distance > 0)
		{
            target_point_ = path_points_[--target_point_index_];
			break;
		}
		last_distance = current_distance;
		
		target_point_index_++;
	}
}

void GridTracking::generateOriginPathPoints(std::vector<gpsMsg_t>& path_points)
{
	gpsMsg_t start_point = current_point_;
	gpsMsg_t end_point = path_vertexes_[0][0];
	
	int points_cnt = point2point_dis(start_point,end_point)/DIS_INCREMENT;
	
	double longitude_increment = (end_point.longitude - start_point.longitude)/points_cnt;
	double latitude_increment  = (end_point.latitude - start_point.latitude)/points_cnt;
	for(size_t i=0; i<points_cnt; i++)
	{
		gpsMsg_t point;
		point.longitude = start_point.longitude + longitude_increment;
		point.latitude = start_point.latitude + latitude_increment;
		path_points.push_back(point);
	}
}

void GridTracking::generateNewPathPoints(uint8_t& dir, std::vector<gpsMsg_t>& path_points)
{
	gpsMsg_t start_point = path_vertexes_[target_vertex_index_.row][target_vertex_index_.col];
	uint8_t row_diff = target_vertex_index_.row-last_vetex_index_.row;
	uint8_t col_diff = target_vertex_index_.col-last_vetex_index_.col;
	
	if(dir == DIR_INVALID)
	{
		//only one selection
		if(target_vertex_index_.row+1 < ROWS && target_vertex_index_.col+1 >= COLS)
			target_vertex_index_.row ++;
		else if(target_vertex_index_.col+1 < COLS && target_vertex_index_.row+1 >= ROWS) 
			target_vertex_index_.col ++;
		else
			return;	
	}
	else if(dir == driverless_msgs::TrafficSign::DirectDir)
	{
		if(row_diff==1 && col_diff==0) //row +
			target_vertex_index_.row ++;
		else if(row_diff==-1 && col_diff==0) //row-
			target_vertex_index_.row --;
		else if(row_diff==0 && col_diff==1) //col+
			target_vertex_index_.col ++;
		else if(row_diff==0 && col_diff==-1) //col-
			target_vertex_index_.col --;
		else
			ROS_ERROR("row_diff or col_diff error!!!");
	}
	else if(dir == driverless_msgs::TrafficSign::LeftDir)
	{
		if(row_diff==1 && col_diff==0) //row +
			target_vertex_index_.col --;
		else if(row_diff==-1 && col_diff==0) //row-
			target_vertex_index_.col ++;
		else if(row_diff==0 && col_diff==1) //col+
			target_vertex_index_.row ++;
		else if(row_diff==0 && col_diff==-1) //col-
			target_vertex_index_.row --;
		else
			ROS_ERROR("row_diff or col_diff error!!!");
	}
	else if(dir == driverless_msgs::TrafficSign::RightDir)
	{
		if(row_diff==1 && col_diff==0) //row +
			target_vertex_index_.col ++;
		else if(row_diff==-1 && col_diff==0) //row-
			target_vertex_index_.col --;
		else if(row_diff==0 && col_diff==1) //col+
			target_vertex_index_.row --;
		else if(row_diff==0 && col_diff==-1) //col-
			target_vertex_index_.row ++;
		else
			ROS_ERROR("row_diff or col_diff error!!!");
	}
	
	gpsMsg_t end_point = path_vertexes_[target_vertex_index_.row][target_vertex_index_.col]; 
	
	else if(dir == driverless_msgs::TrafficSign::DirectDir)
	{
		target_vertex_index_ += COLS;
		start_point = path_vertexes[target_vertex_index_-1];
	}
	else if(dir == driverless_msgs::TrafficSign::LeftDir)
	{
		target_vertex_index_ --;
		start_point = path_vertexes[target_vertex_index_-1];
	}
		
		
		
	end_point = path_vertexes[target_vertex_index_];
			
	int points_cnt = point2point_dis(start_point,end_point)/DIS_INCREMENT;
	
	double longitude_increment = (end_point.longitude - start_point.longitude)/points_cnt;
	double latitude_increment  = (end_point.latitude - start_point.latitude)/points_cnt;
	for(size_t i=0; i<points_cnt; i++)
	{
		gpsMsg_t point;
		point.longitude = start_point.longitude + longitude_increment;
		point.latitude = start_point.latitude + latitude_increment;
		path_points.push_back(point);
	}
	
	dir = DIR_INVALID;
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
		target_point_ = path_points_[target_point_index_];
		std::pair<float, float> dis_yaw = get_dis_yaw(current_point_,target_point_);
		
		if(target_point_index_ > path_points_.size()-3.0/DIS_INCREMENT) //3m
		{
			generateNewPathPoints(traffic_dir_, path_points_);
			
		}
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
