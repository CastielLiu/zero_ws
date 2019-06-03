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
from driverless_msgs.msg import Point
from driverless_msgs.msg import Points
from driverless_msgs.msg import DrawArea

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from dynamic_reconfigure.server import Server
from param_config.cfg import lane_detectConfig

a = range(9)

b = np.array(a)

print(b,type(b))

b = np.reshape(b,(3,3))

print(b,type(b))



b = np.reshape(b,(1,9))[0]
print(b,type(b))
b = list(b)
print(b,type(b))
