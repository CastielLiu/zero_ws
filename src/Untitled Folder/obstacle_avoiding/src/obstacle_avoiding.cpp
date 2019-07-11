#include "obstacle_avoiding.h"




void avoidCallback(const driverless_msgs::lidardata::ConstPtr& data)
{
       Lidar_valid = false;
       Lidar_valid = data->lidar_valid;
       Min_distance = data->min_distance;
       Min_distance_degree = data->min_distance_degree;
       ROS_INFO("Min_distance:%f Min_distance_degree:%f",Min_distance,Min_distance_degree);
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoiding_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<driverless_msgs::lidardata>("lidar_data", 5, avoidCallback);
    ros::Publisher pub = nh.advertise<driverless_msgs::ControlCmd>("lidar_cmd",5);
    ros::NodeHandle nh_private("~");
    nh_private.param<float>("avoiding_disThreshold",avoiding_disThreshold_,1.5);
    nh_private.param<float>("speed",speed_,1.0);
    nh_private.param<float>("steeringAngle",steeringAngle_,20);
    driverless_msgs::ControlCmd lidar_cmd;
    lidar_cmd.set_speed = 0;
    lidar_cmd.set_steeringAngle = 0;
    lidar_cmd.status=false;

    ros::Rate loop_rate(10);
    while(ros::ok())
    { 
    if((Min_distance<=avoiding_disThreshold_) && (Lidar_valid=true))
        {
           lidar_cmd.status=true;
           if(Min_distance_degree>-180&&Min_distance_degree<-90)
           {
            lidar_cmd.set_speed = speed_;
            lidar_cmd.set_steeringAngle = steeringAngle_;
		   }
           if(Min_distance_degree>90&&Min_distance_degree<180)
           {
            lidar_cmd.set_speed = speed_;
            lidar_cmd.set_steeringAngle = -steeringAngle_;
           } 
        }

    else
        {
          lidar_cmd.status=false;
          lidar_cmd.set_speed = 0;
          lidar_cmd.set_steeringAngle = 0;
        }
    pub.publish(lidar_cmd);
    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}
