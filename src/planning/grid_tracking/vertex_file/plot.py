
import matplotlib.pyplot as plt
import math
import sys
 
g_longitudes = []
g_latitudes =[]


file_name = 'raw_path.txt'

if len(sys.argv) >1:
	file_name = sys.argv[1]

with open(file_name,'r') as f:
	lines = f.readlines()
	for line in lines:
		lon,lat = line.split()
		g_longitudes.append(float(lon))
		g_latitudes.append(float(lat))


plt.plot(g_longitudes,g_latitudes,'r-*',lw=1)

plt.show()
