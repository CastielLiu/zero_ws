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
	
	def __init__(self,path):
		rospy.init_node('video_to_ros')
		self.bridge = CvBridge()
		self.pub = rospy.Publisher("/image_rectified",Image,queue_size=1)
		self.videoPath = path
	def run(self):
		while True:
			cap = cv2.VideoCapture(self.videoPath )
			if cap is not None:
				print('open %s ok.' %self.videoPath )
			else:
				print('open %s failed!' %self.videoPath )
				return
				
			while(cap.isOpened()):
				ret,cv_image = cap.read()
				if not ret:
					break
				image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
				self.pub.publish(image_msg)
				time.sleep(0.05)
				if rospy.is_shutdown():
					return
			cap.release()
		
def main(argv):
	if(len(argv)>1):
		video2ros = Video2ROS(argv[1])
		video2ros.run()
	else:
		print('please input the video path')
	
if __name__ == '__main__':
	main(sys.argv)
	


