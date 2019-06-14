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

def read_image(file_name):
	img=cv2.imread(file_name)
	img = cv2.resize(img,(640,480))
	while True:
		publish_image(img)
		cv2.imshow('123',img)
		key=cv2.waitKey(1000)
		if key==ord('q'):
			break
		print("publish")
	

def read_video(video):
	cap = cv2.VideoCapture(video)
	cv2.namedWindow('srcImage')
	while(cap.isOpened()):
		ret,img = cap.read()
		img = cv2.resize(img,(640,480))
		publish_image(img)
		cv2.imshow('srcImage',img)
		key=cv2.waitKey(100)
		if key==ord('q'):
			break
		print("publish")

if __name__ == '__main__':
	#read_video('/home/zwei/wendao/RoadLane_Detect(code_by_kevin)/test1.MOV')
	read_image('bend.jpg')
