#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher puba;

int main(int argc, char** argv) {
    // *****Initialize ROS
    ros::init (argc, argv, "pcd2ros");
    ros::NodeHandle nh;
    
    if(argc <2)
    {
    	ROS_ERROR("please input pcd file path...");
    	return 0;
    }
 
    //*****load two pcd files
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouda_ptr (new pcl::PointCloud<pcl::PointXYZ>);//创建点云指针，存储点坐标xyz
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *clouda_ptr) == -1)
    {
        PCL_ERROR ("Couldn't read file %s ^.^\n",argv[1]);
        return (-1);
    }
    std::cerr << "size of a : " << clouda_ptr->width*clouda_ptr->height<< std::endl;
    
    //*****Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 clouda_ros;
    
    pcl::toROSMsg (*clouda_ptr, clouda_ros);
    
    std::string topic_name = "/point_cloud";
    std::string frame_id = "base_link";
    
   	std::cout << "topic: " << topic_name << std::endl;
   	std::cout << "frame_id: " << frame_id << std::endl;
    
    puba = nh.advertise<sensor_msgs::PointCloud2> (topic_name, 1);
 
    ros::Rate r(3);
    while(ros::ok())
    {
		clouda_ros.header.stamp = ros::Time::now();
		clouda_ros.header.frame_id = frame_id;
		puba.publish(clouda_ros);
		r.sleep();
    }
    return 0;
}
