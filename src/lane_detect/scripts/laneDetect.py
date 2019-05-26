#!/usr/bin/python  
import roslib
import sys
import rospy
import cv2
import math
import os
import numpy as np


def findNearestLinesMsg(img,lines):
	disArray = []
	height = img.shape[0]
	width = img.shape[1]
	for line in lines:
		rho,theta = line[0]
		_cos = np.cos(theta)
		_sin = np.sin(theta)
		x = (rho - height*_sin)/_cos
		disArray.append(x-width/2)
	
	left_index = -1
	right_index = -1
	left_max_dis = -float('inf')
	right_min_dis = float('inf')

	for i in range(len(disArray)):
		if(disArray[i]>0):
			if(disArray[i] < right_min_dis):
				right_min_dis = disArray[i]
				right_index = i
		else:
			if(disArray[i] > left_max_dis):
				left_max_dis = disArray[i]
				left_index = i
				
	if(left_index==-1 or right_index==-1):
		return None
		
	offset = -(right_min_dis+left_max_dis)*disIncrement
	lane_width = (right_min_dis-left_max_dis)*disIncrement
	
	#if lane_width < 2.0:
	#	return None
				
	theta = lines[left_index,0,1] + lines[right_index,0,1]
	
	theta = theta * 180.0/np.pi
	
	if(theta > 90.0):
		theta = theta -180.0
	
	return [left_index,right_index,lane_width,offset,theta]
	
def insertSort(array):
	length = len(array)
	for i in range(length)[1:]:
		anchor = array[i]
		j = i-1
		while True:
			if(j < 0):
				break;
			if(anchor < array[j]):
				array[j+1] = array[j]
			else:
				break;
			j = j-1
		array[j+1] = anchor
		
class Parameters():
	def __init__(self):
		self.__image_height = 480
		self.__image_width = 640
		self.__cut_height = 102
		self.__offset = 100 #pixel
		self.__A = (3,446)
		self.__B = (241,102)
		self.__C = (329,102)
		self.__D = (638,446)
		self.__A_ = (self.__A[0]+self.__offset,self.__image_height-1)
		self.__B_ = (self.__A[0]+self.__offset, 0)
		self.__C_ = (self.__D[0]-self.__offset,0)
		self.__D_ = (self.__D[0]-self.__offset,self.__image_height-1)
		self.__srcPoints = np.float32([self.__A ,self.__B ,self.__C ,self.__D ])
		self.__dstPoints = np.float32([self.__A_,self.__B_,self.__C_,self.__D_])
		self.__transform_M = cv2.getPerspectiveTransform(self.__srcPoints,self.__dstPoints)
		self.__transform_Minv = cv2.getPerspectiveTransform(self.__dstPoints,self.__srcPoints)
		self.__xmPerPixel = 1.0/(self.__D_[0]-self.__A_[0])
		self.__ymPerpixel = 8.60/(self.__D_[1]-self.__C_[1])
		
		print(self.__xmPerPixel,self.__ymPerpixel)
		
	def imageHeight(self):
		return self.__image_height
	def image_width(self):
		return self.__image_width
	def M(self):
		return self.__transform_M
	def Minv(self):
		return self.__transform_Minv
	def cutHeight(self):
		return self.__cut_height
	def xmPerPixel(self):
		return self.__xmPerPixel
	def ymPerPixel(self):
		return self.__ymPerpixel
		
class Point():
	def __init__(self,x,y):
		self.x = x
		self.y = y

class LaneDetect():
	def __init__(self):
		self.__is_debug = False
		self.__params = Parameters()
	def set_debug(self,status):
		self.__is_debug = status
	def deg2rad(self,deg):
		return deg*np.pi/180.0
	def rad2deg(self,rad):
		return rad*180.0/np.pi
	def findNearestLinesIndex(self,img,lines,theta_limit=180.0):
		disArray = []
		height = img.shape[0]
		width = img.shape[1]
	
		for line in lines:
			rho,theta = line[0]
			true_theta = theta
			if theta > np.pi/2:
				true_theta = theta-np.pi
		
			if(true_theta > self.deg2rad(theta_limit) or true_theta < self.deg2rad(-theta_limit)):
				disArray.append(0.0)
				continue
			_cos = np.cos(theta)
			_sin = np.sin(theta)
			x = (rho - height*_sin)/_cos
			disArray.append(x-width/2)
		
		left_index = -1
		right_index = -1
		left_max_dis = -float('inf')
		right_min_dis = float('inf')

		for i in range(len(disArray)):
			if(disArray[i] == 0.0):
				continue
			elif(disArray[i]>0):
				if(disArray[i] < right_min_dis):
					right_min_dis = disArray[i]
					right_index = i
			else:
				if(disArray[i] > left_max_dis):
					left_max_dis = disArray[i]
					left_index = i
				
		if(left_index==-1 or right_index==-1):
			return None
		return [left_index,right_index]
	def get_vertexCoordinates(self,image,lines):
		height = image.shape[0]
		points = []
		for line in lines:
			rho,theta = line[0]
			_cos = np.cos(theta)
			_sin = np.sin(theta)
			point1 = Point(int(rho/_cos),0)
			point2 = Point(int((rho - height*_sin)/_cos), height)
			points.append(point1)
			points.append(point2)
		return points
	def vetexPerspective(self,points):
		result = []
		for point in points:
			point = [point.x,point.y]
			point.append(1)
			src = np.matrix(point).T
			dst = self.__params.M() * src
			res = Point(int(dst[0,0]/dst[2,0]), int(dst[1,0]/dst[2,0]))
			result.append(res)
		return result
	
	def drawLine(self,img,line,color,width=2):
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
	def process(self,srcImage):
		#srcImage = cv2.resize(image,(640,480))
		image = srcImage[self.__params.cutHeight():srcImage.shape[0], : ]  #intresting area
		img_gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
		blur = cv2.GaussianBlur(img_gray, (3, 3), 0)
		canny = cv2.Canny(blur,85,250)
		if self.__is_debug:
			cv2.imshow('canny',canny)
		lines = cv2.HoughLines(canny,1,np.pi/180,140)
		if lines is None:
			return None
		
		indexes = self.findNearestLinesIndex(image,lines,60.0)
		if indexes is None:
			return None
	
		two_line = [lines[index] for index in indexes]
	
		src_points = self.get_vertexCoordinates(image,two_line)
		
		srcImage_points = [Point(point.x,point.y+self.__params.cutHeight()) for point in src_points]
	
		cv2.line(srcImage,(srcImage_points[0].x,srcImage_points[0].y),(srcImage_points[1].x,srcImage_points[1].y),(0,0,255),3)
		cv2.line(srcImage,(srcImage_points[2].x,srcImage_points[2].y),(srcImage_points[3].x,srcImage_points[3].y),(0,0,255),3)
		
		dst_points = self.vetexPerspective(srcImage_points)
		
		if self.__is_debug:
			pure = np.zeros((srcImage.shape[0],srcImage.shape[1],1), np.uint8)
	
			cv2.line(pure,(dst_points[0].x,dst_points[0].y),(dst_points[1].x,dst_points[1].y),(255,255,255),3)
			cv2.line(pure,(dst_points[2].x,dst_points[2].y),(dst_points[3].x,dst_points[3].y),(255,255,255),3)
			cv2.imshow('pure',pure)
			
	
		if(dst_points[1].x < dst_points[3].x):
			left_line = dst_points[0:2]
			right_line = dst_points[2:4]
		else:
			left_line = dst_points[2:4]
			right_line = dst_points[0:2]
		
		latErr = (left_line[1].x + right_line[1].x - image.shape[1])/2
		latErr = latErr*self.__params.xmPerPixel()
		lane_width = (right_line[1].x - left_line[1].x)*self.__params.xmPerPixel()
		
		left_k = 1.0*(left_line[1].x - left_line[0].x)/(left_line[1].y - left_line[0].y)
		left_k = left_k*self.__params.xmPerPixel()/self.__params.ymPerPixel()
		left_theta = math.atan(left_k)
		
		right_k = 1.0*(right_line[1].x - right_line[0].x)/(right_line[1].y - right_line[0].y)
		right_k = right_k*self.__params.xmPerPixel()/self.__params.ymPerPixel()
		right_theta = math.atan(right_k)
		
		theta = (left_theta + right_theta)*180.0/math.pi
	
		return lane_width,latErr,theta
		

def main():
	lane_detect = LaneDetect()
	count = 0
	while True:
		file_name = 'image/'+str(count)+'.jpg'
		image = cv2.imread(file_name)
		if image is None:
			break
		count = count+1
		lane_detect.process(image)
		cv2.waitKey(0)
			

if __name__ =='__main__':
	try:
		main()
	except KeyboardInterrupt:
		exit()

