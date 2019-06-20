#!/usr/bin/python  
import roslib
import sys
import rospy
import cv2
import math
import os
import numpy as np
from autoware_msgs.msg import DetectedObjectArray
from autoware_msgs.msg import DetectedObject
from driverless_msgs.msg import TrafficSign


class ObjectConvert():
	def __init__(self):
		self.pub = rospy.Publisher("/traffic_sign",DetectedObjectArray,queue_size=0)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		self.object_sub = rospy.Subscriber("/detection/image_detector/objects",DetectedObjectArray,self.object_callback)
		self.cvImage = cv2.Mat()
		
	def image_callback(self,rosImage):
		try:
			self.cvImage = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
		except CvBridgeError as e:
			print(e)
			return 
	
	def object_callback(self,in_objects):
		traffic_sign = TrafficSign()
		traffic_sign.traffic_light_validity = True
		traffic_sign.direction_validity = True
		
		for object in in_objects.objects:
			if  not object.invalid:
				continue
			if(object.label == 'Red'):
				traffic_sign.traffic_light = traffic_sign.RedLight
			elif(object.label == 'Green'):
				traffic_sign.traffic_light = traffic_sign.GreenLight
			elif(object.label == 'Straight'):
				traffic_sign.traffic_light = traffic_sign.DirectDir
				
			elif(object.label == 'Left' or object.label == 'Right'):
				traffic_sign.direction = reDetect(object.x,object.y,object.width,object.height)

	def reDetect(self,x,y,width,height):
		LeftImg = img[y:y+height//2, x:x+width//2]
		RightImg = img[y:y+height//2, x+width//2:x+width]
		retl, LeftImg = cv2.threshold(LeftImg, 100, 255, cv2.THRESH_BINARY)
		retr, RightImg = cv2.threshold(RightImg, 100, 255, cv2.THRESH_BINARY)
		# cv2.imshow('LeftImg', LeftImg)
		# cv2.imshow('RightImg', RightImg)
		print(LeftImg.shape)
		lp = len(np.nonzero(LeftImg)[0])
		rp = len(np.nonzero(RightImg)[0])
		if lp > rp:
			return TrafficSign.LeftDir #left
		return TrafficSign.RightDir  #right



def main():
	rospy.init_node('trafficSign_convert')
	

