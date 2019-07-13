

#include "obstacle_detect.h"

using namespace std;



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float x,y,deg,degree,distance;
    lidar_valid=false;
    min_distance=FLT_MAX;
    min_distance_degree=0;
	int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
	for(int i = 0; i<count;i++) 
	{
		if(isinf(scan->ranges[i]))  continue;
        deg = scan->angle_min + scan->angle_increment * i;
		degree = RAD2DEG(deg);
		x = scan->ranges[i] * cos(deg);
		y = scan->ranges[i] * sin(deg);
		if(x>=-2&&x<=0&&y>=-0.4&&y<=0.4)
		{
           distance=sqrt(x * x + y * y);
           if(min_distance>=distance)
           {
            min_distance=distance;
            min_distance_degree=degree;
		   }

        }

	}
    ROS_INFO("x:%f y:%f",min_distance*cos(DEG2RAD(min_distance_degree)),min_distance*sin(DEG2RAD(min_distance_degree)));
    if(min_distance==FLT_MAX)
    lidar_valid=false;
    else
    lidar_valid=true;
    

}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detect_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);
    ros::Publisher pub = nh.advertise<driverless_msgs::lidardata>("lidar_data",5);
    driverless_msgs::lidardata lidar_data;
    lidar_data.lidar_valid=false;
    lidar_data.min_distance=FLT_MAX;
    lidar_data.min_distance_degree=0;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
    lidar_data.lidar_valid=lidar_valid;
    lidar_data.min_distance=min_distance;
    lidar_data.min_distance_degree=min_distance_degree;
    pub.publish(lidar_data);
    ros::spinOnce();
    loop_rate.sleep();
    }    

    

    return 0;
}
