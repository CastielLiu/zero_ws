#!/usr/bin/python  
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import math
import os
import numpy as np
from driverless_msgs.msg import Lane
from driverless_msgs.msg import DrawArea

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from dynamic_reconfigure.server import Server
from param_config.cfg import lane_detectConfig


class image_converter:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		
	def image_callback(self,rosImage):
		try:
			frame = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
			cv2.line(frame,(320,0),(320,480),(0,0,255),2)
			cv2.namedWindow('frame')
			cv2.imshow("frame",frame)
			cv2.waitKey(1)
		except CvBridgeError as e:
			print(e)
			return 

def main(args):
	rospy.init_node('calibration')
	ic = image_converter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
