#include"decision_making.h"




    void lidarCallback(const driverless_msgs::ControlCmd::ConstPtr& lidarcmd)
     {
       cmdMsg_[0].status = lidarcmd->status;
       cmdMsg_[0].set_speed = lidarcmd->set_speed;
       cmdMsg_[0].set_steeringAngle = lidarcmd->set_steeringAngle;
     }


    void gpsCallback(const driverless_msgs::ControlCmd::ConstPtr& gpscmd)
     {
       cmdMsg_[1].status = gpscmd->status;
       cmdMsg_[1].set_speed = gpscmd->set_speed;
       cmdMsg_[1].set_steeringAngle = gpscmd->set_steeringAngle;
     }




int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_making_node");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe<driverless_msgs::ControlCmd>("lidar_cmd", 5, lidarCallback);
    ros::Subscriber gps_sub = nh.subscribe<driverless_msgs::ControlCmd>("gps_cmd", 5, gpsCallback);  
    ros::Publisher pub = nh.advertise<driverless_msgs::ControlCmd>("cmd",5);

    driverless_msgs::ControlCmd cmd;
    for(int i=0;i<SENSOR_NUM;i++)
	{	
		cmdMsg_[i].status = false;
	}
    ros::Rate loop_rate(10);
    while(ros::ok())
    { 
       for(int i=0;i<SENSOR_NUM;i++)
       {
          if(cmdMsg_[i].status == true)
           {
            cmd.set_speed = cmdMsg_[i].set_speed;
            cmd.set_steeringAngle = cmdMsg_[i].set_steeringAngle;
            cmd.status=true;
            break;
           }
          else
           {
            cmd.set_speed = 0;
            cmd.status=false;
           }
       }
    pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}
