#!/usr/bin/python  
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import math
import os
import time
import numpy as np
from driverless_msgs.msg import Lane
from driverless_msgs.msg import DrawArea

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from dynamic_reconfigure.server import Server
from param_config.cfg import lane_detectConfig
from sensor_msgs.msg import CameraInfo
import  matplotlib.pyplot  as plt

#camera params
g_imageSize = [640,480]
g_is_camera_info_ok = False

g_pixel2dis_y = np.array([None]*g_imageSize[1])
g_pixel2dis_x = np.array([g_pixel2dis_y[:]]*g_imageSize[0])

g_pixel2dis = [None]*g_imageSize[0]*g_imageSize[1]*2
g_pixel2dis = np.array(g_pixel2dis).reshape(g_imageSize[0], g_imageSize[1], 2)

def generatePixel2disTable(h,l0,fx,fy,cx,cy):
	global g_pixel2dis_y
	global g_pixel2dis_x
	cy = int(cy + 0.5)
	theta0 = math.atan(h/l0)
	g_pixel2dis_y[cy] = l0
	
	for v in range(g_imageSize[1]):
		alpha = math.atan((cy-v)/fy)
		theta = theta0 - alpha
		y = h/math.tan(theta)
		g_pixel2dis_y[v] = y
		l = math.sqrt(y*y+ h*h)
		
		x_offset = l* (g_imageSize[0]/2-cx)/fx
		for u in range(g_imageSize[0]):
			x = l* (u-cx)/fx - x_offset
			g_pixel2dis_x[u][v] = x 
			g_pixel2dis[u,v] = (x,y)
			#print(x,y)
def dumpPixel2distable(file_name):
	with open(file_name,'w') as f:
		for y in range(len(g_pixel2dis[0])):
			for x in range(len(g_pixel2dis)):
				f.write('%d,%d\t%f,%f\n' %(x,y,g_pixel2dis[x,y,0],g_pixel2dis[x,y,1])) 
	print("dumpPixel2distable in %s ok!" %file_name)

"""
def Pixel2dis(u,v):
	#v dir
	alpha = math.atan((cy-v)/fy)
	theta = theta0-alpha 
	y = h/math.tan(theta)
	l = math.sqrt(y*y+h*h)
	#u dir
	#beta = math.atan((u-cx)/fx)
	#x = l * math.tan(beta)
	x = l* (u-cx)/fx
	return x,y
"""

def drawLine(img,line,color,width=2):
	rho,theta = line[0]
	a = np.cos(theta)
	b = np.sin(theta)
	x0 = rho * a
	y0 = rho * b
	x1 = int(x0 + 1000*(-b))
	y1 = int(y0 + 1000*(a))
	x2 = int(x0 - 1000*(-b))
	y2 = int(y0 - 1000*(a))
	cv2.line(img,(x1,y1),(x2,y2),color,width)
	
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
		
		self.__left_line = Line()
		self.__right_line= Line()
		self.__sobel_thresh_x = (57,255)
		self.__sobel_thresh_y = ()
		self.__h_thresh = ()
		self.__l_thresh = (215,255)
		self.__s_thresh = (164,255)
		
		self.__dir_thresh = (0,255)
		self.__luv_l_thresh = (159,255)
		self.__mag_thresh = (174,255)
		
		self.__debug = False
		self.__calibrate = False
		self.__fitNum = 3
		
		self.lane_msg = Lane()
		
		self.last_leftx_base = -1
		self.last_rightx_base = -1
	
	def setThreshold(self,config):
		self.__sobel_thresh_x = (config.x_sobel_thresh_min,config.x_sobel_thresh_max)
		self.__sobel_thresh_y = (config.y_sobel_thresh_min,config.y_sobel_thresh_max)
		self.__h_thresh = (config.h_thresh_min,config.h_thresh_max)
		self.__l_thresh = (config.l_thresh_min,config.l_thresh_max)
		self.__s_thresh = (config.s_thresh_min,config.s_thresh_max)
		
		#self.__dir_thresh = (config.dir_thresh_min,config.dir_thresh_max)
		self.__luv_l_thresh = (config.luv_l_thresh_min,config.luv_l_thresh_max)
		self.__mag_thresh = (config.mag_thresh_min,config.mag_thresh_max)
	
	def setDebug(self,status):
		self.__debug = status
	def setCaibrate(self,status):
		self.__calibrate = status
	def setFitNum(self,num):
		self.__fitNum = int(num)
		
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
		
	def dir_threshold(self,img, sobel_kernel=3):
		gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		# Calculate the x and y gradients
		sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
		sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
		# Take the absolute value of the gradient direction,
		# apply a threshold, and create a binary image result
		absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
		binary_output = np.zeros_like(absgraddir)
		binary_output[(absgraddir >= self.__dir_thresh[0]) & (absgraddir <= self.__dir_thresh[1])] = 255
		# Return the binary image
		return binary_output
		
	def mag_threshold(self,img, sobel_kernel=3):
		gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		# Take both Sobel x and y gradients
		sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
		sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
		# Calculate the gradient magnitude
		gradmag = np.sqrt(sobelx ** 2 + sobely ** 2)
		# Rescale to 8 bit
		scale_factor = np.max(gradmag) / 255
		gradmag = (gradmag / scale_factor).astype(np.uint8)
		# Create a binary image of ones where threshold is met, zeros otherwise
		binary_output = np.zeros_like(gradmag)
		binary_output[(gradmag >= self.__mag_thresh[0]) & (gradmag <= self.__mag_thresh[1])] = 255
		return binary_output
		
	def luv_select(self, img ,channel='l'):
		luv = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
		
		if(channel=='l'):
			l_channel = luv[:, :, 0]
			binary_output = np.zeros_like(l_channel)
			binary_output[(l_channel > self.__luv_l_thresh[0]) & (l_channel <= self.__luv_l_thresh[1])] = 255
		return binary_output

	def lab_select(self, img, thresh=(0, 255)):
		lab = cv2.cvtColor(img, cv2.COLOR_RGB2Lab)
		b_channel = lab[:,:,2]
		binary_output = np.zeros_like(b_channel)
		binary_output[(b_channel > thresh[0]) & (b_channel <= thresh[1])] = 255
		return binary_output
	
		
	def thresholding(self,img):
	
		"""
		x_thresh = self.abs_sobel_thresh(img, orient='x')
		
		#cv2.line(x_thresh,(self.__image_width,self.__draw_y),(self.__image_width-self.__offset,self.__image_height),(0,0,0),10)
		
		hls_thresh_l = self.hls_select(img,channel='l')
		hls_thresh_s = self.hls_select(img,channel='s')
		
		lab_thresh = self.lab_select(img)
		dir_thresh = self.dir_threshold(img,sobel_kernel=3)
		mag_thresh = self.mag_threshold(img)
		"""
		
		luv_l_thresh = self.luv_select(img,'l')
		
		thresholded = np.zeros_like(img)
		
		#thresholded[(hls_thresh_l == 255) & (hls_thresh_s == 255) | (x_thresh == 255) ]=255
		#thresholded[(hls_thresh_l == 255) | (x_thresh == 255) ]=255
		thresholded[(luv_l_thresh == 255) ]=1
		
		#rect = np.array([ [0,479], [228,152], [449,152], [594,479] ])
		#cv2.fillConvexPoly(thresholded, rect, 0)
		
		rect = np.array([ [80,479], [220,311], [410,311], [514,479] ])
		cv2.fillConvexPoly(thresholded, rect, 0)
		
		#rect = np.array([ [239,304], [239,244], [343,244], [343,304] ])
		#cv2.fillConvexPoly(thresholded, rect, 0)
		
		cv2.circle(thresholded,(305,286),30,(0,0,0),60)
		
		thresholded = cv2.cvtColor(thresholded,cv2.COLOR_RGB2GRAY)
		
		if self.__debug:
			"""
			cv2.namedWindow('x_thresh',0)
			cv2.imshow('x_thresh',x_thresh)
			
			cv2.namedWindow('hls_thresh_l',0)
			cv2.imshow('hls_thresh_l',hls_thresh_l)
			
			cv2.namedWindow('dir_thresh',0)
			cv2.imshow('dir_thresh',dir_thresh)
			
			cv2.namedWindow('mag_thresh',0)
			cv2.imshow('mag_thresh',mag_thresh)
			"""
			
			cv2.namedWindow('luv_l_thresh',0)
			cv2.imshow('luv_l_thresh',luv_l_thresh)
			
			cv2.namedWindow('Binary graph',0)
			cv2.imshow("Binary graph",thresholded*255)
			cv2.waitKey(1)
		
		return thresholded

	#-----------------------------------------main process----------------------------------#
	#---------------------------------------------------------------------------------------#
	def processing(self,frame,show_result=False):
		#wraped = cv2.warpPerspective(frame,self.__M, frame.shape[1::-1], flags=cv2.INTER_LINEAR)
		#thresholded = self.thresholding(wraped)
		thresholded = self.thresholding(frame)
		#thresholded = cv2.warpPerspective(thresholded,self.__M, frame.shape[1::-1], flags=cv2.INTER_LINEAR)
		#cv2.imshow("afterPerspective",thresholded*255)
		
		#lanePixelRange = [32,250] 
		lanePixelRange = [200,340]  #np.int(thresholded.shape[0]/6),480
		#fited by pixels and fited by true distance
		self.find_line(thresholded, lanePixelRange) 
		
		
	def vetexPerspective(self,vertex):
		index = 0
		for point in vertex.points:
			_point = [point.x,point.y,1]
			dst = self.__Minv * (np.matrix(_point).T)
			vertex.points[index].x = int(dst[0,0]/dst[2,0])
			vertex.points[index].y = int(dst[1,0]/dst[2,0])
			index += 1
	
		
	def find_line(self,binary_warped, lanePixelRange):
		
		# Take a histogram of the bottom half of the image
		#histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0) 
		
		ROI = binary_warped[lanePixelRange[0]:lanePixelRange[1], :]
		
		wight = np.array(range(ROI.shape[0])).reshape(ROI.shape[0],1)
		
		histogram = np.sum(ROI*wight, axis=0)
		
		"""
		plt.figure("Histogram")
		plt.cla()
		plt.plot(range(len(histogram)),histogram)
		plt.pause(0.01)
		"""
		
		# Find the peak of the left and right halves of the histogram
		# These will be the starting point for the left and right lines
		midpoint = np.int(histogram.shape[0] / 2)
		leftx_base = np.argmax(histogram[:midpoint])
		rightx_base = np.argmax(histogram[midpoint:]) + midpoint
			
		"""
		basePoint_maxOffset = 50 #pixel
		if(self.last_leftx_base!=-1 and math.fabs(self.last_leftx_base-leftx_base)>basePoint_maxOffset):
			leftx_current = self.last_leftx_base
		else:
			leftx_current = leftx_base
			
		if(self.last_rightx_base!=-1 and math.fabs(self.last_rightx_base-rightx_base)>basePoint_maxOffset):
			rightx_current = self.last_rightx_base
		else:
			rightx_current = rightx_base
		
		self.last_leftx_base = leftx_current
		self.last_rightx_base = rightx_current
		"""
		
		leftx_current = leftx_base
		rightx_current = rightx_base
		
		# Choose the number of sliding windows
		nwindows = 15
		# Set height of windows
		window_height = np.int(ROI.shape[0] / nwindows)
		# Identify the x and y positions of all nonzero pixels in the image
		nonzero = ROI.nonzero()
		nonzeroy = nonzero[0]
		nonzerox = nonzero[1]
	
		# Set the width of the windows +/- margin
		margin = 40
		# Set minimum number of pixels found to recenter window
		minpix = 40
		# Create empty lists to receive left and right lane pixel indices
		left_lane_inds = []
		right_lane_inds = []
		
		leftRectInvalidCount = 0
		rightRectInvalidCount = 0
		startToRecordInvalidRect_left = False
		startToRecordInvalidRect_right = False
		
		rectedROI = ROI.copy()
		# Step through the windows one by one
		for window in range(nwindows):
			# Identify window boundaries in x and y (and right and left)
			win_y_low = ROI.shape[0] - (window + 1) * window_height
			win_y_high = ROI.shape[0] - window * window_height
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
			if(len(good_left_inds)>20 and leftRectInvalidCount<2):
				left_lane_inds.append(good_left_inds)
				startToRecordInvalidRect_left = True
			elif(startToRecordInvalidRect_left):
				leftRectInvalidCount +=1
			elif(window>nwindows/2):
				leftRectInvalidCount = nwindows
				
			if(len(good_right_inds)>20 and rightRectInvalidCount<2):
				right_lane_inds.append(good_right_inds)
				startToRecordInvalidRect_right = True
			elif(startToRecordInvalidRect_right):
				rightRectInvalidCount +=1
			elif(window>nwindows/2):
				rightRectInvalidCount = nwindows
							
			# If you found > minpix pixels, recenter next window on their mean position
			if len(good_left_inds) > minpix:
				tmp_fit = np.polyfit(nonzeroy[good_left_inds], nonzerox[good_left_inds],1)
				leftx_current = np.int(np.polyval(tmp_fit,win_y_low-window_height/2))
			
				#leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
			if len(good_right_inds) > minpix:
				tmp_fit = np.polyfit(nonzeroy[good_right_inds], nonzerox[good_right_inds],1)
				rightx_current = np.int(np.polyval(tmp_fit,win_y_low-window_height/2))
					
			if self.__debug:
				cv2.rectangle(rectedROI,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),1,1)
				cv2.rectangle(rectedROI,(win_xright_low,win_y_low),(win_xright_high,win_y_high),1,1)
		if self.__debug:
			cv2.namedWindow("ROI_rects",0)
			cv2.imshow("ROI_rects",rectedROI*255)
			cv2.waitKey(1)
		
		
		# Concatenate the arrays of indices
		if(len(left_lane_inds)>1):
			left_lane_inds = np.concatenate(left_lane_inds)
		if(len(right_lane_inds)>1):
			right_lane_inds = np.concatenate(right_lane_inds)
		
		
		# Extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds] + lanePixelRange[0]
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds] +lanePixelRange[0]
		
		#print(len(leftx), len(rightx))
		minPixelPerlane = 600
		if(len(leftx) < minPixelPerlane):
			self.lane_msg.left_lane_validity = False
		else:
			self.lane_msg.left_lane_validity = True
		if(len(rightx) < minPixelPerlane):
			self.lane_msg.right_lane_validity = False
		else:
			self.lane_msg.right_lane_validity = True
			
		pixel_fit_l = [0]
		pixel_fit_r = [0]
		if(len(leftx)>2):
			pixel_fit_l  = np.polyfit(lefty , leftx, self.__fitNum)
		if(len(rightx)>2):
			pixel_fit_r = np.polyfit(righty, rightx, self.__fitNum)
		
		if g_is_camera_info_ok:
			dis_fit_l = [0]
			if(len(lefty)>2):
				ly = [g_pixel2dis_y[y] for y in lefty]
				lx = [g_pixel2dis_x[leftx[i],lefty[i]] for i in range(len(leftx))]
				dis_fit_l = np.polyfit(ly,lx, self.__fitNum)
			dis_fit_r = [0]
			if(len(righty)>2):
				ry = [g_pixel2dis_y[y] for y in righty]
				rx = [g_pixel2dis_x[rightx[i],righty[i]] for i in range(len(rightx))]
				dis_fit_r = np.polyfit(ry,rx, self.__fitNum)

			#plot lane in true distance
			if(self.__calibrate):
				plt.figure("True distance fit")
				plt.cla()
				near_dis = g_pixel2dis[g_imageSize[0]//2,lanePixelRange[1],1]
				far_dis = g_pixel2dis[g_imageSize[0]//2,lanePixelRange[0],1]
				y = np.linspace(near_dis,far_dis,50)
				
				if(len(lefty)>2):
					plt.plot(lx,ly,'.')
					x_l = np.polyval(dis_fit_l,y)
					plt.plot(x_l,y,'--')
				
				if(len(righty)>2):
					plt.plot(rx,ry,'.')
					x_r = np.polyval(dis_fit_r,y)
					plt.plot(x_r,y,'--')
				
				plt.grid('on')
				plt.pause(0.01)
		#plot lane in pixels
		"""
		plt.figure("Pixel fit")
		plt.cla()
		plt.plot(leftx,480-lefty,'.')
		plt.plot(rightx,480-righty,'.')
		y = np.array(range(binary_warped.shape[0]))[lanePixelRange[0]:lanePixelRange[1]]
		x_l = np.polyval(pixel_fit_l,y)
		x_r = np.polyval(pixel_fit_r,y)
		plt.plot(x_l,480-y,'--',lw=3)
		plt.plot(x_r,480-y,'--',lw=3)
		plt.pause(0.01)
		"""
		
		self.lane_msg.dimension = len(pixel_fit_l)
		self.lane_msg.pixel_fit_left = pixel_fit_l
		self.lane_msg.pixel_fit_right = pixel_fit_r
		self.lane_msg.dis_fit_left = dis_fit_l
		self.lane_msg.dis_fit_right = dis_fit_r
		self.lane_msg.lanePixelRange = lanePixelRange
		

		
class image_converter:
	def __init__(self,is_reconfig=False):
		
		self.lane_detect = LaneDetect()
		self.lane_detect.setDebug(rospy.get_param('~is_debug'))
		self.lane_detect.setCaibrate(rospy.get_param('~is_calibrate'))
		self.lane_detect.setFitNum(rospy.get_param('~curve_fit_num'))
		
		self.pub_lane_msg = rospy.Publisher("/lane",Lane,queue_size=0)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_rectified",Image,self.image_callback, queue_size=1)
		
		if is_reconfig:
			self.srv = Server(lane_detectConfig, self.config_callback)
		
		
	def image_callback(self,rosImage):
		try:
			frame = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
		except CvBridgeError as e:
			print(e)
			return 
			
		self.lane_detect.processing(frame, show_result=False)
		
		self.pub_lane_msg.publish(self.lane_detect.lane_msg)
		
	def config_callback(self,config, level):
		self.lane_detect.setThreshold(config)
		return config

def cameraInfo_callback(in_message):
	global g_is_camera_info_ok
	
	if g_is_camera_info_ok:
		return
	
	g_is_camera_info_ok = True
	fx = in_message.P[0]
	fy = in_message.P[5]
	cx = in_message.P[2]
	cy = in_message.P[6]
	h = 0.6   #0.575
	l0 = 2.53   #1.457  3.89
	generatePixel2disTable(h,l0,fx,fy,cx,cy)
	dumpPixel2distable('pixel2dis.txt')

def main(args):
	rospy.init_node('lane_detect')
	
	is_reconfig = rospy.get_param('~is_reconfig')
	
	cameraInfo_sub = rospy.Subscriber("/camera_info",CameraInfo, cameraInfo_callback)
	
	if is_reconfig is None:
		ic = image_converter()
	else:
		ic = image_converter(is_reconfig)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
