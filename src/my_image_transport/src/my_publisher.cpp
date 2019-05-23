#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  if(argv[1] == NULL) 
	{
		ROS_ERROR("argv[1]: camera number is empty\n");
		return 1;
	}

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/image_raw", 1);

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
      ROS_ERROR("can not opencv video device\n");
      return 1;
  }
  
  ////////////////////
  cv::Mat frame,src;
  sensor_msgs::ImagePtr msg;
 
 //////////////
 
  ros::Rate loop_rate(30);
  while (nh.ok()) 
  {
    cap >> frame;
    //cv::flip(src,frame,-1); //将摄像头图像翻转一次再发布
    
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) 
    {
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		msg->header.frame_id="camera";
		pub.publish(msg);
		//cv::Wait(1);
    }
    loop_rate.sleep();
  }
}
