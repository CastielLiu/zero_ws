#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"
#include "opencv2/imgproc/detail/distortion_model.hpp"
#include<string>
#include<vector>
#include<iostream>

using namespace std;

#define _NODE_NAME_ "image_mosaic_node"

using namespace cv;

class ImageMosaic
{
public:
	ImageMosaic()
	{
	}
	void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private,std::vector<int>& cameraId)
	{
		camera_id_ = cameraId;
		image_transport::ImageTransport it(nh);
		std::string calibration_file_path;
		nh_private.param<std::string>("calibration_file_path",calibration_file_path,"a.yaml");
		nh_private.param<bool>("is_show_image",is_show_image_,false);
		nh_private.param<int>("frame_rate",frame_rate_,30);
	
		if(!Loadintrinsics(calibration_file_path))
		{
			pub_ = it.advertise("/image_raw", 1);
			is_rectify_ = false;
		}
		else
		{
			pub_ = it.advertise("/image_rectified", 1);
			//new_camera_instrinsics_ = getOptimalNewCameraMatrix(camera_instrinsics_,distortion_coefficients_,imgSize_,1.0);
			is_rectify_ = true;
		}
	}
	
	void run()
	{
		cv::VideoCapture cap0(camera_id_[0]);
		cv::VideoCapture cap1(camera_id_[1]);
		cv::VideoCapture cap2(camera_id_[2]);
		
		if(!cap0.isOpened() ||
			!cap1.isOpened() ||
			!cap2.isOpened())
		{
			ROS_ERROR("can not open video device\n");
			return;
		}
		
		//cap.set(CV_CAP_PROP_FPS, 5);
		//cap.set(CV_CAP_PROP_FRAME_WIDTH, 3840);
		//cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
		
		std::vector<cv::Mat> frame(3);
		sensor_msgs::ImagePtr msg;

		ros::Rate loop_rate(frame_rate_);
		int count = 0 ;
		
		while (ros::ok())
		{
			cap0.grab();
			cap1.grab();
			cap2.grab();
			
			cap0.retrieve(frame[0]);
			cap1.retrieve(frame[1]);
			cap2.retrieve(frame[2]);
			//cv::flip(src,frame,-1); //将摄像头图像翻转一次再发布

			if(!frame[0].empty() && !frame[1].empty() && !frame[2].empty())
			{
				cv::Mat result(frame[0].rows,frame[0].cols*3, CV_8UC3);
				std::cout << result.size() << std::endl;
				
				for(int i=0; i<3; ++i)
				{
					cv::Mat roiImage = result(cv::Rect(640*i,0,640,480));
					frame[i].copyTo(roiImage);
					//cv::imshow(std::string("frame")+std::to_string(i), frame[i]);
				}
				//std::cout << result.size() <<std::endl;
				
				cv::Mat dst;
				cv::flip(result,dst,-1); //将摄像头图像翻转一次再发布
				//cv::imwrite(std::string("/home/wuconglei/Desktop/image/")+std::to_string(count)+".jpg",dst);
				//++ count;
				cv::namedWindow("result",0);
				cv::imshow("result",dst);
				cv::waitKey(1);
				
				/*
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
				if(is_show_image_) 
				{
					cv::namedWindow("image_raw",cv::WINDOW_NORMAL); 
					cv::imshow("image_raw",frame); cv::waitKey(1);
				}
	
				msg->header.frame_id="camera";
				pub_.publish(msg);
				*/
			}
			
			loop_rate.sleep();
		}
	}
	
	bool Loadintrinsics(const std::string &calibration_file_path)
	{
		if (calibration_file_path.empty())
		{
			ROS_ERROR("[%s] missing calibration file path", _NODE_NAME_);
			return false;
		}

		cv::FileStorage fs(calibration_file_path, cv::FileStorage::READ);

		if (!fs.isOpened())
		{
			ROS_INFO("[%s] cannot open calibration file %s", _NODE_NAME_, calibration_file_path.c_str());
	 		return false;
		}

		camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
		distortion_coefficients_ = cv::Mat(1, 5, CV_64F);

		cv::Mat dis_tmp;
		
		fs["CameraMat"] >> camera_instrinsics_;
		fs["DistCoeff"] >> dis_tmp;
		fs["ImageSize"] >> imgSize_;
		fs["DistModel"] >> distModel_;
		
		for (int col = 0; col < 5; col++)
		{
			distortion_coefficients_.at<double>(col) = dis_tmp.at<double>(col);
		}
		fs.release();	//释放
		return true;
	}

private:
	bool is_rectify_;
	image_transport::Publisher pub_ ;
	ros::Publisher camera_info_pub_;
	ros::Timer timer_;
	
	cv::Mat new_camera_instrinsics_;
	cv::Mat camera_instrinsics_;
	cv::Mat distortion_coefficients_;
	std::string distModel_;
	sensor_msgs::CameraInfo camera_info_msg_;
	cv::Size imgSize_;
	std::vector<int> camera_id_;
	int frame_rate_;
	bool is_show_image_;

};


int main(int argc, char** argv)
{
	if(argc !=4) 
	{
		ROS_ERROR("please input 4 camera id. \n");
		return 1;
	}

	ros::init(argc, argv, _NODE_NAME_);
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	std::vector<int> cameraId;
	
	for(int i=0; i<3; i++)
		cameraId.push_back(atoi(argv[1+i]));
		
	ImageMosaic image_mosaic;
	image_mosaic.init(nh,nh_private,cameraId);
	image_mosaic.run();
	return 0;
}
