#!/usr/bin/python  
import roslib
import sys
import rospy
import cv2
import math
import os
import numpy as np


g_debug = True
objectOffset = 100
disIncrement = 1.0/(639-0-2*objectOffset)
image_height = 480
image_width = 640
cut_height = 207


def get_M_Minv():  
    src = np.float32([(0,419-cut_height), (304,0), (367,0), (639,419-cut_height)])
    #dst = np.float32([(86, 453), (86, 276), (552,276), (552,453)])
    #offset = 220/2
    
    dst = np.float32([(0+objectOffset, 419-cut_height), (0+objectOffset, 0), \
    				  (639-objectOffset,0), (639-objectOffset,419-cut_height)])
    
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst,src)
    return M,Minv

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

	#srcImage = cv2.resize(image,(640,480))
	srcImage = image.copy()
	
	#cv2.imshow('rawImage',srcImage)

	image = srcImage[cut_height:image_height, : ]  #intresting area

	img_gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)

	blur = cv2.GaussianBlur(img_gray, (3, 3), 0)

	canny = cv2.Canny(blur,85,250)
	if g_debug:
		cv2.imshow('canny',canny)

	lines = cv2.HoughLines(canny,1,np.pi/180,130)
	
	if lines is None:
		cv2.imshow('result',image)
		return None

	pure = np.zeros((image.shape[0],image.shape[1],1), np.uint8)

	index = findNearestLinesIndex(image,lines,60.0)
	
	if index is None:
		cv2.imshow('result',image)
		print 'index is None'
		return None

	drawLine(pure,lines[index[0]],(255,255,255),1)
	drawLine(pure,lines[index[1]],(255,255,255),1)

	
	
		
	if g_debug:
		cv2.imshow('pure',pure)
	
	
	"""
	kernel = np.ones((2,2),np.uint8)

	dilation = cv2.dilate(canny,kernel,iterations = 2)
	cv2.imshow('dilation',dilation)
	"""

	M,Minv = get_M_Minv()

	top_view = cv2.warpPerspective(pure,M, pure.shape[1::-1], flags=cv2.INTER_LINEAR)
	if g_debug:
		cv2.imshow("top_view",top_view)

	#blur = cv2.GaussianBlur(top_view, (5, 5), 0)
	
	kernel = np.ones((10,2),np.uint8)

	dilation = cv2.dilate(top_view,kernel,iterations = 2)
	cv2.imshow('dilation',dilation)

	canny = cv2.Canny(dilation,150,300)
	if g_debug:
		cv2.imshow('blur2',blur)
		cv2.imshow('canny2',canny)

	lines = cv2.HoughLines(canny,1,np.pi/180,90)
	
	if lines is None:
		cv2.imshow('result',image)
		return None
		
	laneMsgs = findNearestLinesMsg(image,lines)  #left_index,right_index,lane_width,offset,theta
	
	if laneMsgs is None:
		cv2.imshow('result',image)
		return None

	pure = np.zeros((image.shape[0],image.shape[1],image.shape[2]), np.uint8)
	
	left_index = laneMsgs[0]
	right_index = laneMsgs[1]
	
	drawLine(pure,lines[left_index],(0,0,255),5)
	drawLine(pure,lines[right_index],(0,0,255),5)
	if g_debug:
		cv2.imshow('pure2',pure)

	original_view = cv2.warpPerspective(pure,Minv, pure.shape[1::-1], flags=cv2.INTER_LINEAR)

	for row in range(original_view.shape[0]):
		for col in range(original_view.shape[1]):
			if(original_view[row,col,2]==255):
				image[row,col,2] = 255
				image[row,col,1] = 0
				image[row,col,0] = 0

	cv2.imshow('result',image)
	
	return laneMsgs[2:]

def main():
	count = 0
	while True:
		file_name = 'image/'+str(count)+'.jpg'
		image = cv2.imread(file_name)
		if image is None:
			break
		count = count+1
		print(laneDetect(image))
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
