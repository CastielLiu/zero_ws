#!/usr/bin/python  
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import math
import os
import numpy as np

from dynamic_reconfigure.server import Server
from param_config.cfg import lane_detectConfig

def callback(config, level):
	rospy.loginfo(config.s_thresh_min)
	return config
		
if __name__ == "__main__":
	rospy.init_node("test", anonymous = False)
	srv = Server(lane_detectConfig, callback)
	rospy.spin()

