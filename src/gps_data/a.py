
import matplotlib.pyplot as plt
import math
import sys
 
g_longitudes = []
g_latitudes =[]
def main(argv):
	if(len(argv)==2):
		file_name1 = argv[1]
		with open(file_name1,'r') as f:
			lines = f.readlines()
	
		for line in lines:
			lon,lat = line.split()
			g_longitudes.append(float(lon))
			g_latitudes.append(float(lat))


	plt.plot(g_latitudes,g_longitudes,'r--',label="reference path")
	plt.plot(g_latitudes,g_longitudes,'b--',label="trajectory")
		
	plt.legend()
	plt.savefig('a.pdf')
	plt.show()
"""
g_longitudes = []
g_latitudes =[]

with open('debug.txt','r') as f:
	lines = f.readlines()
	
for line in lines:
	lon,lat = line.split()
	g_longitudes.append(float(lon))
	g_latitudes.append(float(lat))


plt.plot(g_latitudes,g_longitudes,'b--')
"""

	
	
if __name__=="__main__":
	main(sys.argv)
