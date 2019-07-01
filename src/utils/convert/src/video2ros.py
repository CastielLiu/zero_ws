#!/usr/bin/python

import rospy
import roslib
import sys
import thread

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class Video2ROS():
	
	def __init__(self,argv):
		rospy.init_node('video_to_ros')
		self.bridge = CvBridge()
		self.pub = rospy.Publisher("/image_rectified",Image,queue_size=1)
		self.videoPath = argv[1]
		self.frameRate = int(argv[2])
		self.is_waitKey = int(argv[3])
		self.image_msg = None
		self.cv_image = None
		print('video path: %s' %self.videoPath)
		print('frameRate: %d' %self.frameRate)
		print('is waitKey: %d' %self.is_waitKey)
		try:
			thread.start_new_thread(self.publishVideo, ("publishVideoThread",) )
		except:
			print "Error: unable to start thread"
			
	def readVideo(self,capture):
		while(capture.isOpened() and not rospy.is_shutdown()):
			ret,self.cv_image = capture.read()
			if not ret:
				break
			self.image_msg = self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
			self.image_msg.header.stamp = rospy.Time().now()
			cv2.namedWindow("image",0)
			cv2.imshow("image",self.cv_image)
			
			if self.is_waitKey:
				key = cv2.waitKey(0)
			else:
				key = cv2.waitKey(int(1000.0/self.frameRate))
			#print(self.is_waitKey , key)
			if(113 == key):
				rospy.signal_shutdown("quit")
			if(114 == key): #r  restart
				break
			
	def publishVideo(self,threadName):
		duration = 1.0/self.frameRate
		while not rospy.is_shutdown():
			if self.image_msg is not None:
				self.pub.publish(self.image_msg)
			time.sleep(duration)
			
			
		
def main(argv):
	if(len(argv)==4):
		video2ros = Video2ROS(argv)
		while not rospy.is_shutdown():
			cap = cv2.VideoCapture(video2ros.videoPath )
			if cap is not None:
				print('open %s ok.' %video2ros.videoPath )
			else:
				print('open %s failed!' %video2ros.videoPath )
				return
			video2ros.readVideo(cap)
			cap.release()
			
	else:
		print('please input the (videoPath,frameRate,is_waitKey)')

if __name__ == '__main__':
	try:
		main(sys.argv)
	except KeyboardInterrupt:
		print("node over...")


