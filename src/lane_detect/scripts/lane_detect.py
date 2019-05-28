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

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from dynamic_reconfigure.server import Server
from param_config.cfg import lane_detectConfig


# Define a class to receive the characteristics of each line detection
class Line():
	def __init__(self):
		# was the line detected in the last iteration?
		self.detected = False
		# x values of the last n fits of the line
		self.recent_fitted = [np.array([False])]
		# average x values of the fitted line over the last n iterations
		self.bestx = None
		# polynomial coefficients averaged over the last n iterations
		self.best_fit = None
		# polynomial coefficients for the most recent fit
		self.current_fit = [np.array([False])]
		# radius of curvature of the line in some units
		self.radius_of_curvature = None
		# distance in meters of vehicle center from the line
		self.line_base_pos = None
		# difference in fit coefficients between last and new fits
		self.diffs = np.array([0, 0, 0], dtype='float')
		# x values for detected line pixels
		self.allx = None
		# y values for detected line pixels
		self.ally = None

	def check_detected(self):
		if (self.diffs[0] < 0.01 and self.diffs[1] < 10.0 and self.diffs[2] < 1000.) and len(self.recent_fitted) > 0:
			return True
		else:
			return False

	def update(self, fit):
		if fit is not None:
			if self.best_fit is not None:
				self.diffs = abs(fit - self.best_fit)
				if self.check_detected():
					self.detected = True
					if len(self.recent_fitted) > 10:
					    self.recent_fitted = self.recent_fitted[1:]
					    self.recent_fitted.append(fit)
					else:
					    self.recent_fitted.append(fit)
					self.best_fit = np.average(self.recent_fitted, axis=0)
					self.current_fit = fit
				else:
					self.detected = False
			else:
				self.best_fit = fit
				self.current_fit = fit
				self.detected = True
				self.recent_fitted.append(fit)

#------------------------find line-------------------------------------------#

class LaneDetect():
	def __init__(self):
		self.__image_height = 480
		self.__image_width = 640
		self.__cut_height = 229				##@param
		self.__offset = 50 #pixel
		self.__A = (91,478)					##@param
		self.__B = (301,self.__cut_height)	##@param
		self.__C = (396,self.__cut_height)	##@param
		self.__D = (575,478)				##@param
		self.__A_ = (self.__A[0]+self.__offset,self.__image_height-1)
		self.__B_ = (self.__A[0]+self.__offset, 0)
		self.__C_ = (self.__D[0]-self.__offset,0)
		self.__D_ = (self.__D[0]-self.__offset,self.__image_height-1)
		self.__srcPoints = np.float32([self.__A ,self.__B ,self.__C ,self.__D ])
		self.__dstPoints = np.float32([self.__A_,self.__B_,self.__C_,self.__D_])
		self.__M = cv2.getPerspectiveTransform(self.__srcPoints,self.__dstPoints)
		self.__Minv = cv2.getPerspectiveTransform(self.__dstPoints,self.__srcPoints)
		self.__xmPerPixel = 0.9/(self.__D_[0]-self.__A_[0])		##@param
		self.__ymPerpixel = 4.40/(self.__D_[1]-self.__C_[1])	##@param
		self.__left_line = Line()
		self.__right_line= Line()
		self.__sobel_thresh_x = (54,,255)
		self.__sobel_thresh_y = ()
		self.__h_thresh = ()
		self.__l_thresh = (120,255)
		self.__s_thresh = (0,255)
		
		self.__debug = False
	
	def setThreshold(self,config):
		self.__sobel_thresh_x = (config.x_sobel_thresh_min,config.x_sobel_thresh_max)
		self.__sobel_thresh_y = (config.y_sobel_thresh_min,config.y_sobel_thresh_max)
		self.__h_thresh = (config.h_thresh_min,config.h_thresh_max)
		self.__l_thresh = (config.l_thresh_min,config.l_thresh_max)
		self.__s_thresh = (config.s_thresh_min,config.s_thresh_max)
	
	def setDebug(self,status):
		self.__debug = status
		
	def abs_sobel_thresh(self,img,orient='x'):
		gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
		if orient == 'x':
			abs_sobel = np.absolute(cv2.Sobel(gray,cv2.CV_64F,1,0))
			scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
			binary_output = np.zeros_like(scaled_sobel)
			binary_output[(scaled_sobel >= self.__sobel_thresh_x[0]) & (scaled_sobel <=self.__sobel_thresh_x[1])] = 255
		elif orient =='y':
			abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
			scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
			binary_output = np.zeros_like(scaled_sobel)
			binary_output[(scaled_sobel >= self.__sobel_thresh_y[0]) & (scaled_sobel <=self.__sobel_thresh_y[1])] = 255
		return binary_output

	def hls_select(self,img,channel='s'):
		hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
		if channel == 'h':
			channel = hls[:, :, 0]
			binary_output = np.zeros_like(channel)
			binary_output[(channel > self.__h_thresh[0]) & (channel <= self.__h_thresh[1])] = 255
		elif channel == 'l':
			channel = hls[:, :, 1]
			binary_output = np.zeros_like(channel)
			binary_output[(channel > self.__l_thresh[0]) & (channel <= self.__l_thresh[1])] = 255
		else:
			channel = hls[:, :, 2]
			binary_output = np.zeros_like(channel)
			binary_output[(channel > self.__s_thresh[0]) & (channel <= self.__s_thresh[1])] = 255
		return binary_output
		
	def thresholding(self,img):
		x_thresh = self.abs_sobel_thresh(img, orient='x')
	
		#cv2.imshow("scaled_sobel",binary_output)
		hls_thresh_white = self.hls_select(img,channel='l')
		hls_thresh_yellow = self.hls_select(img,channel='s')
		#cv2.imshow("scaled_sobel",hls_thresh_yellow)
		thresholded = np.zeros_like(x_thresh)
		thresholded[(hls_thresh_white == 255) & (hls_thresh_yellow == 255) | (x_thresh == 255) ]=255
		
		if self.__debug:
			cv2.imshow('x_thresh',x_thresh)
			cv2.imshow('hls_thresh_white',hls_thresh_white)
			cv2.imshow('hls_thresh_yellow',hls_thresh_yellow)
			cv2.imshow("thresholded",thresholded)
		
		return thresholded

	#-----------------------------------------main process----------------------------------#
	#---------------------------------------------------------------------------------------#
	def processing(self,frame):
		
		wraped = cv2.warpPerspective(frame,self.__M, frame.shape[1::-1], flags=cv2.INTER_LINEAR)
		thresholded = self.thresholding(wraped)
		if self.__left_line.detected and self.__right_line.detected:
			left_fit, right_fit = self.find_line_by_previous(thresholded)
		else:
			left_fit, right_fit = self.find_line(thresholded)

		lane_width,pos_from_center,angle,validity = self.calculate_lane_state(left_fit, right_fit)
		
		area_img = self.draw_area(frame,left_fit,right_fit)
		result = self.draw_values(area_img,pos_from_center,angle)
		cv2.imshow("result",result)
		cv2.waitKey(1)
			
		msg = Lane()
		msg.validity = validity
		msg.lane_width = lane_width
		msg.offset = -pos_from_center
		msg.theta = angle
		return msg
		
	def find_line(self,binary_warped):
		# Take a histogram of the bottom half of the image
		histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0) 
		# Find the peak of the left and right halves of the histogram
		# These will be the starting point for the left and right lines
		midpoint = np.int(histogram.shape[0] / 2)
		leftx_base = np.argmax(histogram[:midpoint])
		rightx_base = np.argmax(histogram[midpoint:]) + midpoint

		# Choose the number of sliding windows
		nwindows = 9
		# Set height of windows
		window_height = np.int(binary_warped.shape[0] / nwindows)
		# Identify the x and y positions of all nonzero pixels in the image
		nonzero = binary_warped.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		# Current positions to be updated for each window
		leftx_current = leftx_base
		rightx_current = rightx_base
		# Set the width of the windows +/- margin
		margin = 100
		# Set minimum number of pixels found to recenter window
		minpix = 30
		# Create empty lists to receive left and right lane pixel indices
		left_lane_inds = []
		right_lane_inds = []

		# Step through the windows one by one
		for window in range(nwindows):
			# Identify window boundaries in x and y (and right and left)
			win_y_low = binary_warped.shape[0] - (window + 1) * window_height
			win_y_high = binary_warped.shape[0] - window * window_height
			win_xleft_low = leftx_current - margin
			win_xleft_high = leftx_current + margin
			win_xright_low = rightx_current - margin
			win_xright_high = rightx_current + margin
			# Identify the nonzero pixels in x and y within the window
			good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
						      (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
			good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
						       (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
			# Append these indices to the lists
			left_lane_inds.append(good_left_inds)
			right_lane_inds.append(good_right_inds)
			# If you found > minpix pixels, recenter next window on their mean position
			if len(good_left_inds) > minpix:
				leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
			if len(good_right_inds) > minpix:
				rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

		# Concatenate the arrays of indices
		left_lane_inds = np.concatenate(left_lane_inds)
		right_lane_inds = np.concatenate(right_lane_inds)
		
		# Extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds]

		# Fit a second order polynomial to each
		left_fit  = np.polyfit(lefty , leftx, 1)
		right_fit = np.polyfit(righty, rightx, 1)

		return left_fit, right_fit 


	def find_line_by_previous(self,binary_warped, left_fit, right_fit):
		nonzero = binary_warped.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		margin = 100
		left_lane_inds = ((nonzerox > (left_fit[0] * nonzeroy + left_fit[1] - margin)) & (nonzerox < (left_fit[0] * nonzeroy +
		                                                                     left_fit[1] + margin)))

		right_lane_inds = ((nonzerox > (right_fit[0] * nonzeroy + right_fit[1] - margin)) & (nonzerox < (right_fit[0] * nonzeroy +
		                                                                       right_fit[1] + margin)))

		# Again, extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds]
		# Fit a second order polynomial to each
		left_fit = np.polyfit(lefty, leftx, 1)
		right_fit = np.polyfit(righty, rightx, 1)
		return left_fit, right_fit 


	def calculate_lane_state(self,left_fit, right_fit):
		left_angle=math.atan(left_fit[0]) * self.__xmPerPixel/self.__ymPerpixel
		right_angle=math.atan(right_fit[0])*self.__xmPerPixel/self.__ymPerpixel
		
		angle=((left_angle+right_angle)/2)
	
		left_bottom_x = np.polyval(left_fit, self.__image_height)
		right_bottom_x = np.polyval(right_fit, self.__image_height)
	
		x_offset_pixel = 1.0*(left_bottom_x + right_bottom_x - self.__image_width)/2
		
		lane_width = (right_bottom_x - left_bottom_x) * self.__xmPerPixel;
		
		distance_from_center = x_offset_pixel * self.__xmPerPixel;
		
		if (left_angle-right_angle)*180.0/math.pi >5.0 or distance_from_center > 0.5:
			print('two angle diff or distance_from_center is too big')
			validity = False
		else:
			validity = True

		return lane_width,distance_from_center,angle,validity

	def draw_area(self,undist,left_fit,right_fit):
	
		Ax = np.polyval(left_fit, 0)
		Bx = np.polyval(right_fit,0)
		Cx = np.polyval(right_fit,undist.shape[0])
		Dx = np.polyval(left_fit,undist.shape[0])

		A = [Ax,0]
		B = [Bx,0]
		C = [Cx,undist.shape[0]]
		D = [Dx,undist.shape[0]]

		rect = np.array([[A,B,C,D]]).astype(np.int)
		pure = np.zeros_like(undist)
		cv2.fillPoly(pure, rect, (0, 255, 0))

		# Warp the blank back to original image space using inverse perspective matrix (Minv)
		newwarp = cv2.warpPerspective(pure, self.__Minv, (undist.shape[1], undist.shape[0]))
		# Combine the result with the original image
		result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
		return result

	def draw_values(self,img, distance_from_center,angle):
		font = cv2.FONT_HERSHEY_SIMPLEX
		#radius_text = "Radius of Curvature: %sm" % (round(curvature))

		if distance_from_center > 0:
			pos_flag = 'right'
		else:
			pos_flag = 'left'

		#cv2.putText(img, radius_text, (100, 100), font, 1, (255,0,255), 2)
		center_text = "Vehicle is %.3fm %s of center" % (abs(distance_from_center), pos_flag)
		cv2.putText(img, center_text, (100, 150), font, 1, (255, 0, 255), 2)
		#langle_text="left angle is:%.3f"%(left_angle*180/math.pi)
		#cv2.putText(img,langle_text,(100,200),font,1,(255,0,255),2)
		#rangle_text="right angle is:%.3f"%(right_angle*180/math.pi)
		#cv2.putText(img,rangle_text,(100,250),font,1,(255,0,255),2)
		angle_text="angle is %.3f"%(angle*180/math.pi)
		cv2.putText(img,angle_text,(100,200),font,1,(255,0,255),2)
		return img
class image_converter:
	def __init__(self):
		self.lane_detect_method = LaneDetect()
		self.lane_detect_method.setDebug(True)
		
		self.image_pub = rospy.Publisher("/lane",Lane,queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		self.srv = Server(lane_detectConfig, self.config_callback)
		
		
	def image_callback(self,rosImage):
		try:
			frame = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
		except CvBridgeError as e:
			print(e)
			return 
		lane_msg = self.lane_detect_method.processing(frame)
		self.image_pub.publish(lane_msg)
		
	def config_callback(self,config, level):
		self.lane_detect_method.setThreshold(config)
		return config
	
	

def main(args):
	rospy.init_node('lane_detect')
	ic = image_converter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
