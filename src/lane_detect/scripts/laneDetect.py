#!/usr/bin/python  
import roslib
import sys
import rospy
import cv2
import math
import os
import numpy as np

g_debug = False

def get_M_Minv():  
    src = np.float32([(86,177), (263,0), (349,0), (552,177)])
    #dst = np.float32([(86, 453), (86, 276), (552,276), (552,453)])
    #offset = 220/2
    offset = 0
    dst = np.float32([(86+offset, 204), (86+offset, 0), (552-offset,0), (552-offset,204)])
    
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
	
	left_index = 0
	right_index = 0
	left_max_dis = -width/2
	right_min_dis = width/2

	for i in range(len(disArray)):
		if(disArray[i]>0):
			if(disArray[i] < right_min_dis):
				right_min_dis = disArray[i]
				right_index = i
		else:
			if(disArray[i] > left_max_dis):
				left_max_dis = disArray[i]
				left_index = i
				
	theta = lines[left_index,0,1] + lines[right_index,0,1]
	
	theta = theta * 180.0/np.pi
	
	if(theta > 90.0):
		theta = theta - 180.0
	
	return left_index,right_index,left_max_dis,right_min_dis,theta
	
def laneDetect(image):	

	srcImage = cv2.resize(image,(640,480))

	image = srcImage[276:480, : ]  #intresting area

	img_gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)

	blur = cv2.GaussianBlur(img_gray, (3, 3), 0)

	canny = cv2.Canny(blur,120,300)
	if g_debug:
		cv2.imshow('canny',canny)

	lines = cv2.HoughLines(canny,1,np.pi/180,135)

	pure = np.zeros((image.shape[0],image.shape[1],1), np.uint8)

	for line in lines:
		drawLine(pure,line,(255,255,255),2)
		
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

	blur = cv2.GaussianBlur(top_view, (3, 3), 0)

	canny = cv2.Canny(blur,100,300)
	if g_debug:
		cv2.imshow('canny2',canny)

	lines = cv2.HoughLines(canny,1,np.pi/180,80)

	left_index,right_index,left_dis,right_dis,theta = findNearestLinesMsg(image,lines)
	
	print('left :%d\t%d' %(left_index,left_dis))
	print('right:%d\t%d' %(right_index,right_dis))

	pure = np.zeros((image.shape[0],image.shape[1],image.shape[2]), np.uint8)

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
	
	return left_dis,right_dis,theta



def main():
	image = cv2.imread('lane.jpg')
	
	while True:
		print(laneDetect(image))
		if(cv2.waitKey(500) != -1):
			break
			
				
if __name__ =='__main__':
	main()

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
