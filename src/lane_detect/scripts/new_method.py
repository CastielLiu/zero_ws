#!/usr/bin/python  
import roslib
import sys
import rospy
import cv2
import math
import os
import numpy as np

g_cut_height = 200
g_image_height = 480
g_debug = True


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

def rad2deg(rad):
	return rad*180.0/np.pi

def deg2rad(deg):
	return deg*np.pi/180.0


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

def findNearestLinesIndex(img,lines,theta_limit=180.0):
	disArray = []
	height = img.shape[0]
	width = img.shape[1]
	
	for line in lines:
		rho,theta = line[0]
		true_theta = theta
		if theta > np.pi/2:
			true_theta = theta-np.pi
		
		if(true_theta > deg2rad(theta_limit) or true_theta < deg2rad(-theta_limit)):
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


def laneDetect(image):

	srcImage = image.copy()
	
	#cv2.imshow('rawImage',srcImage)
	global g_cut_height
	global g_image_height
	image = srcImage[g_cut_height:g_image_height, : ]  #intresting area

	img_gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)

	blur = cv2.GaussianBlur(img_gray, (3, 3), 0)

	canny = cv2.Canny(blur,85,250)
	if g_debug:
		cv2.imshow('canny',canny)

	lines = cv2.HoughLines(canny,1,np.pi/180,120)
	
	if lines is None:
		cv2.imshow('result',image)
		return None

	index = findNearestLinesIndex(image,lines,60.0)
	
	if index is None:
		cv2.imshow('result',image)
		print 'index is None'
		return None

	drawLine(image,lines[index[0]],(0,0,255),5)
	drawLine(image,lines[index[1]],(0,0,255),5)
	cv2.imshow('result',image)
	"""
	kernel = np.ones((2,2),np.uint8)

	dilation = cv2.dilate(canny,kernel,iterations = 2)
	cv2.imshow('dilation',dilation)
	"""


def main():
	count = 0
	while True:
		file_name = 'image/'+str(count)+'.jpg'
		image = cv2.imread(file_name)
		if image is None:
			break
		count = count+1
		laneDetect(image)
		print cv2.waitKey(0)
		#cv2.destroyAllWindows()
			
				
if __name__ =='__main__':
	try:
		main()
	except KeyboardInterrupt:
		exit()
"""
for col in range(image.shape[1]):
	for channel in range(image.shape[2]):
		for offset in range(10):
			image[462-offset,col,channel] = 0

for col in range(image.shape[1]):
	for channel in range(image.shape[2]):
		for offset in range(10):
			image[276-offset,col,channel] = 0
"""			
