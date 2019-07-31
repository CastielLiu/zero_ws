#include<ros/ros.h>
#include<tf/transform_datatypes.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/Imu.h>
#include <pcl/point_types.h>

class Vector3
{
public:
	Vector3(float _x, float _y, float _z):
		x(_x),
		y(_y),
		z(_z){}

	Vector3(void):
		x(0),
		y(0),
		z(0){}
	Vector3 operator*(double val)
	{
		return Vector3(this->x*val, this->y*val, this->z*val);
	}
	
	Vector3 operator+(Vector3 vec)
	{
		return Vector3(vec.x+this->x,vec.y+this->y,vec.z+this->z);
	}
	
	friend Vector3 operator*(double val, Vector3 _vector);
	double x,y,z;
};

Vector3 operator*(double val, Vector3 _vector)
{
	return Vector3(_vector.x*val, _vector.y*val, _vector.z*val);
}
  
struct IMUState {
  /** The time of the measurement leading to this state (in seconds). */
  ros::Time stamp;

  /** The current roll angle. */
  double roll;

  /** The current pitch angle. */
  double pitch;

  /** The current yaw angle. */
  double yaw;

  /** The accumulated global IMU position in 3D space. */
  Vector3 position;

  /** The accumulated global IMU velocity in 3D space. */
  Vector3 velocity;

  /** The current (local) IMU acceleration in 3D space. */
  Vector3 acceleration;
  
  bool is_empty;
  
};

template<typename T>
void rotateZXY(T &point, double angle_z, double angle_x, double angle_y)
{
	point.x = cos(angle_z) * point.x - sin(angle_z) * point.y;
	point.y = sin(angle_z) * point.x + cos(angle_z) * point.y;
	
	point.x = cos(angle_x) * point.x - sin(angle_x) * point.y;
	point.y = sin(angle_x) * point.x + cos(angle_x) * point.y;
	
	point.x = cos(angle_y) * point.x - sin(angle_y) * point.y;
	point.y = sin(angle_y) * point.x + cos(angle_y) * point.y;
}


class ImuOdom
{
public:
	ImuOdom()
	{
		prevState_.is_empty = true;
	}
	~ImuOdom(){}
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nh_private;
		std::string imu_topic,odom_topic;
		nh_private.param<std::string>("imu_topic",imu_topic,"/imu");
		nh_private.param<std::string>("odom_topic",odom_topic,"/odom");
		
		sub_imu_ = nh.subscribe(imu_topic,100,&ImuOdom::imu_handler,this);
		pub_odom_ = nh.advertise<nav_msgs::Odometry>(odom_topic,10);
		return true;
	}
	
	void imu_handler(const sensor_msgs::Imu::ConstPtr& imuIn)
	{
		double roll, pitch, yaw;
		tf::Quaternion orientation;
		tf::quaternionMsgToTF(imuIn->orientation, orientation);
		tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

		Vector3 acc;
		acc.x = float(imuIn->linear_acceleration.x + sin(pitch) * 9.81);
		acc.y = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
		acc.z = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);

		IMUState newState;
		newState.stamp = imuIn->header.stamp;
		newState.roll = roll;
		newState.pitch = pitch;
		newState.yaw = yaw;
		newState.acceleration = acc;

		if(prevState_.is_empty) 
		{
			prevState_ = newState;
			prevState_.is_empty = false;
			return ;
		}
		
		// accumulate IMU position and velocity over time
		rotateZXY(acc, newState.yaw, newState.roll, newState.pitch);

		float timeDiff = float((newState.stamp - prevState_.stamp).toSec());
		newState.position = prevState_.position
				            + (prevState_.velocity * timeDiff)
				            + (0.5 * acc * timeDiff * timeDiff);
		newState.velocity = prevState_.velocity
				            + acc * timeDiff;
		ROS_INFO("position:%f\t%f\t%f",newState.position.x,newState.position.y,newState.position.z);

	}
private:
	ros::Subscriber sub_imu_;
	ros::Publisher pub_odom_;
	IMUState prevState_;
};


int main(int argc,char **argv)
{
	ros::init(argc,argv,"imu_odom_node");
	ImuOdom imu_odom;
	imu_odom.init();
	ros::spin();
}

