
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


	plt.plot(g_latitudes,g_longitudes,'r--')
	plt.plot(g_latitudes[70],g_longitudes[70],'b*')
	plt.plot(g_latitudes[110],g_longitudes[110],'r*')
	
	plt.plot(g_latitudes[160],g_longitudes[160],'b*')
	plt.plot(g_latitudes[190],g_longitudes[190],'r*')
	
	plt.plot(g_latitudes[240],g_longitudes[240],'b*')
	plt.plot(g_latitudes[270],g_longitudes[270],'r*')
	
	plt.plot(g_latitudes[325],g_longitudes[325],'b*')
	plt.plot(g_latitudes[360],g_longitudes[360],'r*')
		
		
	
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

plt.savefig('a.pdf')
"""
	
	
if __name__=="__main__":
	main(sys.argv)
