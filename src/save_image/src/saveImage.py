#!/usr/bin/python

import rospy
import roslib

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

count = 10000
imagePath = 'sign/'

class SaveImage():
	
	def __init__(self):
		rospy.init_node('save_image')
		self.bridge = CvBridge()
		self.sub_image = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		
	def run(self):
		rospy.spin()
		
	def image_callback(self,image):
		global count
		try:
			frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)
		file_name = imagePath + str(count)+'.jpg'
		status = cv2.imwrite(file_name, frame)
		if status:
			print(file_name+' saved')
		else:
			print('save image failed!!')
		count = count+1
		
def main():
	save_image = SaveImage()
	save_image.run()
	
if __name__ == '__main__':
	main()
	


