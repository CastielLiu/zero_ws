#!/usr/bin/python  

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import os
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


rospy.init_node("publisher", anonymous=True)

image_pubulish=rospy.Publisher('image_rectified',Image,queue_size=1)

bridge = CvBridge()

def publish_image(imgdata):
	image_temp= bridge.cv2_to_imgmsg(imgdata,'bgr8')
	
	image_pubulish.publish(image_temp)
	
while True:
	img=cv2.imread('bend.jpg')
	publish_image(img)
	cv2.imshow('123',img)
	key=cv2.waitKey(1000)
	if key==ord('q'):
		break
	print("pubulish")
