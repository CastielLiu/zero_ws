
import matplotlib.pyplot as plt
import math
import sys
import numpy as np
 

def BLH2XYZ(lon,lat,H=0.0):
	B = lat*math.pi/180.0
	L = lon*math.pi/180.0
	a = 6378136.49
	b = 6356755.00
	e2 = (a*a-b*b)/(a*a)
	sinB = math.sin(B)
	cosB = math.cos(B)
	N = a/math.sqrt(1-e2*sinB*sinB)
	X = (N+H)*cosB*math.cos(L)
	Y = (N+H)*cosB*math.sin(L)
	Z = (N*(1-e2)+H)*sinB
	return X,Y,Z


file_name = 'raw_path.txt'

if len(sys.argv) >1:
	file_name = sys.argv[1]

g_longitudes = []
g_latitudes =[]
with open(file_name,'r') as f:
	lines = f.readlines()
	for line in lines:
		lon,lat = line.split()
		g_longitudes.append(float(lon))
		g_latitudes.append(float(lat))

plt.figure(1)
plt.plot(g_longitudes,g_latitudes,'r-*',lw=1)

g_longitudes = []
g_latitudes =[]
file_name = 'trajectory.txt'
with open(file_name,'r') as f:
	lines = f.readlines()
	for line in lines:
		lon,lat = line.split()
		g_longitudes.append(float(lon))
		g_latitudes.append(float(lat))

plt.plot(g_longitudes,g_latitudes,'b-*',lw=1,label="trajectory")
plt.legend()

plt.show()
