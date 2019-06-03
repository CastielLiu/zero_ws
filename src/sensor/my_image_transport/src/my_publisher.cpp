#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

#define _NODE_NAME_ "image_publisher"

namespace ImageRectifier{

bool Loadintrinsics(const std::string &calibration_file_path,
					cv::Mat& camera_instrinsics, 
					cv::Mat& distortion_coefficients)
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

	camera_instrinsics = cv::Mat(3, 3, CV_64F);
	distortion_coefficients = cv::Mat(1, 5, CV_64F);

	cv::Mat dis_tmp;
	fs["CameraMat"] >> camera_instrinsics;
	fs["DistCoeff"] >> dis_tmp;
	//fs["ImageSize"] >> image_size;

	for (int col = 0; col < 5; col++)
	{
	    distortion_coefficients.at<double>(col) = dis_tmp.at<double>(col);
	}
	fs.release();	//释放
	return true;
}

}

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
	if(argv[1] == NULL) 
	{
		ROS_ERROR("argv[1]: camera number is empty\n");
		return 1;
	}

	ros::init(argc, argv, _NODE_NAME_);
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub ;
	
	std::string calibration_file_path;
	nh_private.param<std::string>("calibration_file_path",calibration_file_path,"a.yaml");
	
	int frame_rate_;
	nh_private.param<int>("frame_rate",frame_rate_,30);
	
	ROS_INFO("%s",calibration_file_path.c_str());
	
	cv::Mat camera_instrinsics;
	cv::Mat distortion_coefficients;
	
	bool is_rectify ;
	if(!ImageRectifier::Loadintrinsics(calibration_file_path,camera_instrinsics,distortion_coefficients))
	{
		pub = it.advertise("/image_raw", 1);
		is_rectify = false;
	}
	else
	{
		pub = it.advertise("/image_rectified", 1);
		is_rectify = true;
	}
	
	// Convert the passed as command line parameter index for the video device to an integer
	std::istringstream video_sourceCmd(argv[1]);
	int video_source;
	// Check if it is indeed a number
	if(!(video_sourceCmd >> video_source)) 
	{
		ROS_INFO("video_sourceCmd is %d\n",video_source);
		return 1;
	}

	cv::VideoCapture cap(video_source);
	// Check if video device can be opened with the given index
	if(!cap.isOpened()) 
	{
		ROS_ERROR("can not open video device\n");
		return 1;
	}
	
	cv::Mat frame,src;
	sensor_msgs::ImagePtr msg;

	ros::Rate loop_rate(frame_rate_);
	while (nh.ok())
	{
		cap >> frame;
		//cv::flip(src,frame,-1); //将摄像头图像翻转一次再发布

		if(!frame.empty()) 
		{
			if(is_rectify)
			{
				cv::undistort(frame, src, camera_instrinsics, distortion_coefficients);
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
			}
			else
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			msg->header.frame_id="camera";
			pub.publish(msg);
		}
		loop_rate.sleep();
	}
}