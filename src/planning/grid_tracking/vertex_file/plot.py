
import matplotlib.pyplot as plt
import math
 
g_longitudes = []
g_latitudes =[]

with open('data.txt','r') as f:
	lines = f.readlines()
	for line in lines:
		lon,lat = line.split()
		g_longitudes.append(float(lon))
		g_latitudes.append(float(lat))


plt.plot(g_latitudes,g_longitudes,'ro',lw=10)

g_longitudes = []
g_latitudes =[]

with open('result.txt','r') as f:
	lines = f.readlines()
	for line in lines:
		lon,lat = line.split()
		g_longitudes.append(float(lon))
		g_latitudes.append(float(lat))


plt.plot(g_latitudes,g_longitudes,'b--')


plt.show()
