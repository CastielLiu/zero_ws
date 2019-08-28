#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>

using PointT = pcl::PointXYZI;

ros::Publisher puba;

pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative)
{
    pcl::PlaneClipper3D<PointT> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);

    return dst_cloud;
}
  
  

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
    pcl::PointCloud<PointT>::Ptr cloud_raw (new pcl::PointCloud<PointT>);//创建点云指针，存储点坐标xyz
    if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud_raw) == -1)
    {
        PCL_ERROR ("Couldn't read file %s ^.^\n",argv[1]);
        return (-1);
    }
    std::cerr << "size of a : " << cloud_raw->width*cloud_raw->height<< std::endl;
    
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    float sensor_height = 1.8;
    filtered = plane_clip(cloud_raw, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + 0.0), false);
    filtered = plane_clip(cloud_raw, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - 1), true);
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
	viewer->setBackgroundColor (0, 0, 0); 
	viewer->addPointCloud<PointT> (filtered, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    
    //*****Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 clouda_ros;
    
    pcl::toROSMsg (*filtered, clouda_ros);
    
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
		viewer->spinOnce (100);
		r.sleep();
    }
    return 0;
}
