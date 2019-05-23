#!/usr/bin/python

from laneDetect import *
import rospy
import roslib

from driverless_msgs.msg import Lane
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

count = 0
imagePath = '/home/wendao/projects/zero_ws/src/lane_detect/scripts/image/'

class LaneDetect():
	
	def __init__(self):
		rospy.init_node('save_image')
		self.lane_msg = Lane()
		self.pub_lane_msg = rospy.Publisher("/lane",Lane,queue_size=1)
		self.bridge = CvBridge()
		self.sub_image = rospy.Subscriber("/image_raw",Image,self.image_callback)
		
	def run(self):
		rospy.spin()
		
	def image_callback(self,image):
		global count
		try:
			frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		cv2.imwrite(imagePath + str(count)+'.jpg',frame)
		count = count+1
		
def main():
	_lane_detect = LaneDetect()
	_lane_detect.run()
	
if __name__ == '__main__':
	main()
	


