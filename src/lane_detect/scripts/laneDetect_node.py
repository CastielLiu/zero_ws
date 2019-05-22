#!/usr/bin/python

from laneDetect import *
import rospy
import roslib

from driverless_msgs.msg import Lane
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LaneDetect():
	def __init__(self):
		rospy.init_node('laneDetect_node')
		self._lane = Lane()
		self.pub_lane_msg = rospy.Publisher("/lane",Lane,queue_size=1)
		self.bridge = CvBridge()
		self.sub_image = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		
	def run(self):
		rospy.spin()
		
	def image_callback(self,image):
		try:
			frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)
		frame = cv2.resize(frame,(640,480))
		
		left_dis,right_dis,theta = laneDetect(frame)
		
		
def main():
	_lane_detect = LaneDetect()
	_lane_detect.run()
	
if __name__ == '__main__':
	main()
	


