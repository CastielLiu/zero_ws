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

class Image2ROS():
	
	def __init__(self,argv):
		rospy.init_node('image_to_ros')
		self.bridge = CvBridge()
		self.pub = rospy.Publisher("/image_rectified",Image,queue_size=1)
		self.imagePath = argv[1]
		self.frameRate = int(argv[2])
		self.is_waitKey = int(argv[3])
		
	def run(self):
		image_names = os.listdir(self.imagePath)
		
		while not rospy.is_shutdown():
			for image_name in image_names:
				image_file = self.imagePath + '/' + image_name
				
				if os.path.isdir(image_file) or (image_file.split('.')[-1] != 'jpg') :
					continue
					
				print('%s  published' %image_file)
				
				cv_image = cv2.imread(image_file)				
				image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
				self.pub.publish(image_msg)
				if rospy.is_shutdown():
					return
					
				cv2.imshow("image",cv_image)
				if self.is_waitKey:
					key = cv2.waitKey(0)
				else:
					key = cv2.waitKey(int(1000.0/self.frameRate))
				if(113 == key):
					return
		
def main(argv):
	if(len(argv)>1):
		image2ros = Image2ROS(argv)
		image2ros.run()
	else:
		print('please input the file path')
	
if __name__ == '__main__':
	main(sys.argv)
	


