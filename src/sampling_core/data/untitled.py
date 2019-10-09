import numpy as np
import math
import random

min_lat = 40.443576
max_lat = 40.443657
min_lng = -79.943069
max_lng = -79.943009
resolution = 0.00001

gps_file = open("random_GPS.txt", "w")
temp_file = open("random_temperature.txt","w")

for lat in np.arange(min_lat, max_lat, resolution):
	for lng in np.arange(min_lng, max_lng, resolution):
		gps_file.write("%f , %f \n" %(lat,lng))
		temp_file.write("%f\n" % (-11.90 + random.uniform(-1,1)))

gps_file.close()
temp_file.close()	