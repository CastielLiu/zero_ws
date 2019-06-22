#!/usr/bin/python  
import roslib
import sys
import rospy
import cv2
import math
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from autoware_msgs.msg import DetectedObjectArray
from autoware_msgs.msg import DetectedObject
from driverless_msgs.msg import TrafficSign


class ObjectConvert():
	def __init__(self):
		self.pub = rospy.Publisher("/traffic_sign",DetectedObjectArray,queue_size=0)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		self.object_sub = rospy.Subscriber("/detection/image_detector/objects",DetectedObjectArray,self.object_callback)
		self.cvImage = None
		self.cvImageStamp = rospy.Time().now()
		
	def image_callback(self,rosImage):
		try:
			self.cvImage = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
			self.cvImageStamp = rosImage.header.stamp
		except CvBridgeError as e:
			print(e)
			return
	
	def object_callback(self,in_objects):
		if self.cvImage is None:
			return
			
		print(in_objects.header.stamp.to_nsec())
		print(self.cvImageStamp.to_nsec())
		
		traffic_sign = TrafficSign()
		traffic_sign.traffic_light_validity = True
		traffic_sign.direction_validity = True
		
		for object in in_objects.objects:
			if not object.valid:
				continue
				
			if(object.label == 'Red'):
				traffic_sign.traffic_light = TrafficSign.RedLight
			elif(object.label == 'Green'):
				traffic_sign.traffic_light = TrafficSign.GreenLight
			elif(object.label == 'Straight'):
				traffic_sign.direction = TrafficSign.DirectDir
				
			elif(object.label == 'Left' or object.label == 'Right'):
				traffic_sign.direction = self.reDetect(object.x,object.y,object.width,object.height)
			
			print(object.label , traffic_sign.direction)
		print('\n')

	def reDetect(self,x,y,width,height):
		scope = width//3
		LeftImg = self.cvImage[y:y+height, x+width//2-scope:x+width//2]
		RightImg = self.cvImage[y:y+height, x+width//2:x+width//2+scope]
		retl, LeftImg = cv2.threshold(LeftImg, 100, 255, cv2.THRESH_BINARY)
		retr, RightImg = cv2.threshold(RightImg, 100, 255, cv2.THRESH_BINARY)
		cv2.imshow('LeftImg', LeftImg)
		cv2.imshow('RightImg', RightImg)
		cv2.waitKey(1)
		#print(LeftImg.shape)
		lp = len(np.nonzero(LeftImg)[0])
		rp = len(np.nonzero(RightImg)[0])
		if lp > rp:
			return TrafficSign.LeftDir #left
		return TrafficSign.RightDir  #right

	def spin(self):
		rospy.spin()

def main():
	rospy.init_node('trafficSign_convert')
	object_convert = ObjectConvert()
	object_convert.spin()
	
if __name__ == '__main__':
	main()
