 #include "ros/ros.h"  
#include "image_transport/image_transport.h"  
#include "cv_bridge/cv_bridge.h"  
#include "sensor_msgs/image_encodings.h"  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <iostream>  
      
namespace enc = sensor_msgs::image_encodings;  
  
  
class ImageConvertor  
{  
    ros::NodeHandle nh_;  
    image_transport::ImageTransport it_;  
    image_transport::Subscriber image_sub_;  
    image_transport::Publisher image_pub_;  
      
    public:  
    ImageConvertor():it_(nh_){  
        /*发布主题out*/  
        image_pub_ = it_.advertise("/image_raw", 1); 
                /*订阅主题camera/image*/  
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConvertor::ImageCb, this);  
       
    }  
  
    ~ImageConvertor()  
    {  
      
    }  
  
    void ImageCb(const sensor_msgs::ImageConstPtr& msg)  
    {  
        cv_bridge::CvImagePtr cv_ptr;  
  		 sensor_msgs::ImagePtr image_msg;
  		 
        try  
        {  
             /*转化成CVImage*/  
                     cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);  
                    
        }   
        catch (cv_bridge::Exception& e)  
        {  
            ROS_ERROR("cv_bridge exception is %s", e.what());  
            return;  
        }  
  
	  	cv::Mat src = cv_ptr->image;
		cv::Mat dst;
		cv::flip(src,dst,-1);
  
      image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
                /*转化成ROS图像msg发布到主题out*/            
    image_msg->header.frame_id="camera";
     
        image_pub_.publish(image_msg);  
    }  
  
  
};  
  
  
int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "image_node_b");  
    ImageConvertor ic;  
    ros::spin();  
      
    return 0;  
}

