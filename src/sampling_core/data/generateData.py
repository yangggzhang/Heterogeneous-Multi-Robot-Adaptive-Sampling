import numpy as np
import math
import random

latitude1 = 40.000000
longitude1 = -79.000000

latitude2 = 40.000040
longitude2 = -79.000040

scale = 1000000.0

min_lat = min(latitude1, latitude2)
max_lat = max(latitude1, latitude2)
min_lng = min(longitude1, longitude2)
max_lng = max(longitude1, longitude2)
resolution = 0.000001

print("latitude_range: [" + str(min_lat) + ", " + str(max_lat) + "]")
print("longitude_range: [" + str(min_lng) + ", " + str(max_lng) + "]")

gps_file = open("groundtruth_GPS.txt", "w")
temp_file = open("groundtruth_temperature.txt","w")

heat_source_lat = [40.000010, 40.000030]
heat_source_lng = [-79.000010, -79.000030]

ambient_temp = 5.0
heat_temp = 30.0
max_distance = 0.00001

def distance(lat1, lng1, lat2, lng2):
	d_lat = lat1 - lat2
	d_lng = lng1 - lng2
	return np.sqrt(d_lat * d_lat + d_lng * d_lng)

for lat in np.arange(min_lat, max_lat + resolution, resolution):
	for lng in np.arange(min_lng, max_lng + resolution, resolution):
		temperature = ambient_temp + random.uniform(-2, 2)
		for heat_lat, heat_lng in zip(heat_source_lat, heat_source_lng):
			d = distance(lat, lng, heat_lat, heat_lng)
			if d <= max_distance:
				temperature = temperature + (max_distance - d)/max_distance * (heat_temp - ambient_temp)
				print(temperature)
		gps_file.write("%f , %f \n" %(lat * scale, lng * scale))
		temp_file.write("%f\n" % (temperature))
			

gps_file.close()
temp_file.close()	
