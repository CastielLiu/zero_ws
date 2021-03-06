#!/usr/bin/python

import rospy
import roslib
import sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class SaveImage():
	
	def __init__(self,path,start_num):
		rospy.init_node('save_image')
		self.bridge = CvBridge()
		self.sub_image = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		self.imagePath = path
		self.start_num = start_num
		
	def run(self):
		rospy.spin()
		
	def image_callback(self,image):
		try:
			frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)
		file_name = self.imagePath + str(self.start_num)+'.jpg'
		status = cv2.imwrite(file_name, frame)
		if status:
			print(file_name+' saved')
		else:
			print('save image failed!!')
		self.start_num = self.start_num+1
		
def main(argv):
	if(len(argv)>1):
		save_image = SaveImage(argv[1],int(argv[2]))
		save_image.run()
	
if __name__ == '__main__':
	main(sys.argv)
	


