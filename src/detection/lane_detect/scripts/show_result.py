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
	
		
class ShowResult:
	def __init__(self):
		self.bridge = CvBridge()
		self.sub_image = rospy.Subscriber("/image_rectified",Image,self.image_callback, queue_size=1)
		self.sub_lane = rospy.Subscriber("/lane",Lane,self.lane_callback,  queue_size=1)
		self.lane_msg = Lane()
		self.image = Image()
		self.is_save_video = False
		if(self.is_save_video):
			self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
			self.videoOut = cv2.VideoWriter("result.avi",self.fourcc, 20.0, (640,480))
		
	def draw_area(self,img,area):
		Minv = np.array(area.Minv)
		Minv = np.reshape(Minv,(3,3))
		
		print(Minv,type(Minv))
		
		A = [np.polyval(area.left_fit, 0),0]
		B = [np.polyval(area.right_fit,0),0]
		C = [np.polyval(area.right_fit,img.shape[0]),img.shape[0]]
		D = [np.polyval(area.left_fit,img.shape[0]),img.shape[0]]

		rect = np.array([[A,B,C,D]]).astype(np.int)
		pure = np.zeros_like(img)
		cv2.fillPoly(pure, rect, (0, 255, 0))
	
		#cv2.imshow("ss",pure)
		
		newwarp = cv2.warpPerspective(pure, Minv, (img.shape[1], img.shape[0]))
		
		#cv2.imshow("newwarp",newwarp)
		return cv2.addWeighted(img, 1, newwarp, 0.3, 0)



	def draw_values(self,img, offset,angle):
		font = cv2.FONT_HERSHEY_SIMPLEX
		if offset > 0:
			pos_flag = 'right'
		else:
			pos_flag = 'left'

		center_text = "Vehicle is %.3fm %s of center" % (abs(offset), pos_flag)
		cv2.putText(img, center_text, (100, 150), font, 1, (255, 0, 255), 2)
		
		angle_text="angle is %.3f"%(angle*180/math.pi)
		cv2.putText(img,angle_text,(100,200),font,1,(255,0,255),2)
	
	def lane_callback(self,lane):
		self.lane_msg = lane
		pixel_fit_left = lane.pixel_fit_left
		pixel_fit_right = lane.pixel_fit_right
		
		pureImage = np.zeros_like(self.image)
		#left
		y1 = lane.lanePixelRange[0] 
		y2 = lane.lanePixelRange[1] 
		
		y = np.linspace(y1,y2,30)
		
		left_x = np.polyval(lane.pixel_fit_left, y)
		right_x = np.polyval(lane.pixel_fit_right, y)
		for i in range(len(y)):
			if(lane.left_lane_validity):
				cv2.circle(pureImage,(int(left_x[i]), int(y[i])),5,(0,255,0), -1)
			if(lane.right_lane_validity):
				cv2.circle(pureImage,(int(right_x[i]), int(y[i])),5,(0,255,0), -1)
		#cv2.imshow("pureImage",pureImage)
		result = cv2.addWeighted(self.image, 1, pureImage, 0.8, 0)
		if(self.is_save_video):
			self.videoOut.write(result)
		cv2.namedWindow("result",0)
		cv2.imshow("result",result)
		cv2.waitKey(1)
	
	def image_callback(self,rosImage):
		try:
			self.image = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
		except CvBridgeError as e:
			print(e)

def main(args):
	rospy.init_node('show_result_node')
	show_result = ShowResult()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
