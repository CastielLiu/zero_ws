#!/usr/bin/python

import rospy
import roslib
import sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class Video2ROS():
	
	def __init__(self,argv):
		rospy.init_node('ros2video_node')
		self.bridge = CvBridge()
		self.subImage = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		self.videoPath = argv[1]+'/video.avi'
		self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
		self.videoOut = cv2.VideoWriter(self.videoPath,self.fourcc, 20.0, (640,480))
		
		print('video will saved in %s' %self.videoPath)
		
	def image_callback(self, rosImage):
		try:
			frame = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
			self.videoOut.write(frame)
		except CvBridgeError as e:
			print(e)
			return 
			
def main(argv):
	if(len(argv)>1):
		ros2video = Video2ROS(argv)
		try:
			rospy.spin()
		except KeyboardInterrupt:
			ros2video.videoOut.release()
		
	else:
		print('please input the video path')

if __name__ == '__main__':
	main(sys.argv)
	


