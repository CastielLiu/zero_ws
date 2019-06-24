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
		self.__cut_height = 0				##@param
		self.__draw_y = 355					##@param  
		self.__offset = 180 #pixel
		self.__angleOffset = 1.1/180.0*np.pi##@param 
		self.__A = (10,328)					##@param
		self.__B = (278,self.__cut_height)	##@param
		self.__C = (378,self.__cut_height)	##@param
		self.__D = (639,328)				##@param
		self.__A_ = (self.__A[0]+self.__offset,self.__image_height-1)
		self.__B_ = (self.__A[0]+self.__offset, 0)
		self.__C_ = (self.__D[0]-self.__offset,0)
		self.__D_ = (self.__D[0]-self.__offset,self.__image_height-1)
		self.__srcPoints = np.float32([self.__A ,self.__B ,self.__C ,self.__D ])
		self.__dstPoints = np.float32([self.__A_,self.__B_,self.__C_,self.__D_])
		self.__M = cv2.getPerspectiveTransform(self.__srcPoints,self.__dstPoints)
		self.__Minv = cv2.getPerspectiveTransform(self.__dstPoints,self.__srcPoints)
		self.__xmPerPixel = 0.90/(self.__D_[0]-self.__A_[0])		##@param
		self.__ymPerpixel = 5.4/(self.__D_[1]-self.__C_[1])	##@param
		self.__left_line = Line()
		self.__right_line= Line()
		self.__sobel_thresh_x = (57,255)
		self.__sobel_thresh_y = ()
		self.__h_thresh = ()
		self.__l_thresh = (215,255)
		self.__s_thresh = (164,255)
		
		self.__dir_thresh = (0,255)
		self.__luv_l_thresh = (180,200)
		self.__mag_thresh = (0,255)
		
		self.__debug = False
		self.area = DrawArea()
		self.area.Minv =  list(np.reshape(self.__Minv,(1,9))[0])
	
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
		
	def luv_select(self, img):
		luv = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
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
		#x_thresh = self.abs_sobel_thresh(img, orient='x')
		#cv2.line(x_thresh,(self.__image_width,self.__draw_y),(self.__image_width-self.__offset,self.__image_height),(0,0,0),10)
	
		hls_thresh_l = self.hls_select(img,channel='l')
		#hls_thresh_s = self.hls_select(img,channel='s')
		
		
		#dir_thresh = self.dir_threshold(img,sobel_kernel=3)
		mag_thresh = self.mag_threshold(img)
		luv_thresh = self.luv_select(img)
		
		#lab_thresh = self.lab_select(img)
		
		thresholded = np.zeros_like(img)
		
		#thresholded[(hls_thresh_l == 255) & (hls_thresh_s == 255) | (x_thresh == 255) ]=255
		#thresholded[(hls_thresh_l == 255) | (x_thresh == 255) ]=255
		thresholded[(mag_thresh == 255) | (luv_thresh == 255) ]=1
		
		rect = np.array([ [0,479], [0,340], [248,180], [392,180],[639,340], [639,479] ])
		cv2.fillConvexPoly(thresholded, rect, 0)
		
		thresholded = cv2.cvtColor(thresholded,cv2.COLOR_RGB2GRAY)
		
		cv2.line(thresholded,(0,self.__draw_y),(self.__offset,self.__image_height),(0,0,0),20)
		cv2.line(thresholded,(self.__image_width,self.__draw_y),(self.__image_width - self.__offset,self.__image_height),(0,0,0),20)
		
		if self.__debug:
			cv2.imshow("top2down",img)
			#cv2.imshow('x_thresh',x_thresh)
			cv2.imshow('hls_thresh_l',hls_thresh_l)
			
			#cv2.imshow('dir_thresh',dir_thresh)
			cv2.imshow('mag_thresh',mag_thresh)
			cv2.imshow('luv_thresh',luv_thresh)
			
			cv2.imshow("thresholded",thresholded*255)
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
		
		#fited by pixels and fited by true distance
		pixel_fit_l, pixel_fit_r, dis_fit_l, dis_fit_r = self.find_line(thresholded, cut_height=0 ) #np.int(thresholded.shape[0]/6)
		
		lane_msg = Lane()
		
		lane_msg.validity = True
		lane_msg.dimension = len(pixel_fit_l)
		lane_msg.pixel_fit_left = pixel_fit_l
		lane_msg.pixel_fit_right = pixel_fit_r
		lane_msg.dis_fit_left = dis_fit_l
		lane_msg.dis_fit_right = dis_fit_r
		
		return lane_msg
		
	def vetexPerspective(self,vertex):
		index = 0
		for point in vertex.points:
			_point = [point.x,point.y,1]
			dst = self.__Minv * (np.matrix(_point).T)
			vertex.points[index].x = int(dst[0,0]/dst[2,0])
			vertex.points[index].y = int(dst[1,0]/dst[2,0])
			index += 1
	
		
	def find_line(self,binary_warped, cut_height):
		# Take a histogram of the bottom half of the image
		#histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0) 
		
		ROI = binary_warped[cut_height:, :]
		
		wight = np.array(range(ROI.shape[0])).reshape(ROI.shape[0],1)
		
		histogram = np.sum(ROI*wight, axis=0)
		
		"""
		plt.figure(3)
		plt.cla()
		plt.plot(range(len(histogram)),histogram)
		plt.pause(0.01)
		"""
		
		# Find the peak of the left and right halves of the histogram
		# These will be the starting point for the left and right lines
		midpoint = np.int(histogram.shape[0] / 2)
		leftx_base = np.argmax(histogram[:midpoint])
		rightx_base = np.argmax(histogram[midpoint:]) + midpoint
		
		#print(leftx_base,rightx_base)

		# Choose the number of sliding windows
		nwindows = 9
		# Set height of windows
		window_height = np.int(ROI.shape[0] / nwindows)
		# Identify the x and y positions of all nonzero pixels in the image
		nonzero = ROI.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		# Current positions to be updated for each window
		leftx_current = leftx_base
		rightx_current = rightx_base
		# Set the width of the windows +/- margin
		margin = 30
		# Set minimum number of pixels found to recenter window
		minpix = 30
		# Create empty lists to receive left and right lane pixel indices
		left_lane_inds = []
		right_lane_inds = []

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
			left_lane_inds.append(good_left_inds)
			right_lane_inds.append(good_right_inds)
			
			#print(len(good_left_inds), len(good_right_inds))
			
			# If you found > minpix pixels, recenter next window on their mean position
			if len(good_left_inds) > minpix:
				tmp_fit = np.polyfit(nonzeroy[good_left_inds], nonzerox[good_left_inds],1)
				leftx_current = np.int(np.polyval(tmp_fit,win_y_low-window_height/2))
			
				#leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
			if len(good_right_inds) > minpix:
				tmp_fit = np.polyfit(nonzeroy[good_right_inds], nonzerox[good_right_inds],1)
				rightx_current = np.int(np.polyval(tmp_fit,win_y_low-window_height/2))
				
			cv2.rectangle(ROI,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),1,1)
			cv2.rectangle(ROI,(win_xright_low,win_y_low),(win_xright_high,win_y_high),1,1)
		#cv2.imshow("ROI_rects",ROI*255)

		# Concatenate the arrays of indices
		left_lane_inds = np.concatenate(left_lane_inds)
		right_lane_inds = np.concatenate(right_lane_inds)
		
		# Extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds] + cut_height
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds] +cut_height

		pixel_fit_l  = np.polyfit(lefty , leftx, 2)
		pixel_fit_r = np.polyfit(righty, rightx, 2)
		
		if g_is_camera_info_ok:
			ly = [g_pixel2dis_y[y] for y in lefty]
			lx = [g_pixel2dis_x[leftx[i],lefty[i]] for i in range(len(leftx))]
			dis_fit_l = np.polyfit(ly,lx, 2)
			
			ry = [g_pixel2dis_y[y] for y in righty]
			rx = [g_pixel2dis_x[rightx[i],righty[i]] for i in range(len(rightx))]
			dis_fit_r = np.polyfit(ry,rx, 2)
		"""
			#plot lane in true distance
			plt.figure(0)
			plt.cla()
			plt.plot(lx,ly,'.')
			plt.plot(rx,ry,'.')
			y = np.linspace(0,8,80)
			x_l = np.polyval(dis_fit_l,y)
			x_r = np.polyval(dis_fit_r,y)
			
			plt.plot(x_l,y,'--')
			plt.plot(x_r,y,'--')
			plt.grid('on')
			plt.pause(0.01)
		
		#plot lane in pixels
		plt.figure(1)
		plt.cla()
		plt.plot(leftx,480-lefty,'.')
		plt.plot(rightx,480-righty,'.')
		y = np.array(range(binary_warped.shape[0]))[cut_height:]
		x_l = np.polyval(pixel_fit_l,y)
		x_r = np.polyval(pixel_fit_r,y)
		plt.plot(x_l,480-y,'--',lw=3)
		plt.plot(x_r,480-y,'--',lw=3)
		plt.pause(0.01)
		"""
		
		return pixel_fit_l, pixel_fit_r, dis_fit_l, dis_fit_r


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
		
		if (left_angle-right_angle)*180.0/math.pi >20.0 or distance_from_center > 0.5:
			print('two angle diff or offset is too big -> left:%.2f\t right:%.2f\t offset:%.2f' \
						%(left_angle*180.0/np.pi,right_angle*180.0/np.pi,distance_from_center))
			validity = False
		else:
			validity = True

		return lane_width,distance_from_center,angle,validity

	def draw_area(self,undist,left_fit,right_fit,is_project=True):
	
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
		if(is_project):
			# Warp the blank back to original image space using inverse perspective matrix (Minv)
			pure = cv2.warpPerspective(pure, self.__Minv, (undist.shape[1], undist.shape[0]))
		# Combine the result with the original image
		result = cv2.addWeighted(undist, 1, pure, 0.3, 0)
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
	def __init__(self,is_reconfig=False):
		self.lane_detect = LaneDetect()
		#self.lane_detect.setDebug(True)
		
		self.pub_lane_msg = rospy.Publisher("/lane",Lane,queue_size=0)
		self.pub_draw_area = rospy.Publisher("/draw_area",DrawArea,queue_size=0)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_rectified",Image,self.image_callback)
		
		if is_reconfig:
			self.srv = Server(lane_detectConfig, self.config_callback)
		
		
	def image_callback(self,rosImage):
		try:
			frame = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
		except CvBridgeError as e:
			print(e)
			return 
			
		lane_msg = self.lane_detect.processing(frame, show_result=False)
		
		self.pub_lane_msg.publish(lane_msg)
		
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
	l0 = 1.50  #1.56
	generatePixel2disTable(h,l0,fx,fy,cx,cy)
	dumpPixel2distable('/home/wuconglei/a_wendao/zero_ws/pixel2dis.txt')

def main(args):
	rospy.init_node('lane_detect')
	is_debug = rospy.get_param('~is_debug')
	is_reconfig = rospy.get_param('~is_reconfig')
	
	cameraInfo_sub = rospy.Subscriber("/camera_info",CameraInfo, cameraInfo_callback)
	
	if is_reconfig is None:
		ic = image_converter()
	else:
		ic = image_converter(is_reconfig)
	
	if is_debug is not None:
		ic.lane_detect.setDebug(is_debug)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
