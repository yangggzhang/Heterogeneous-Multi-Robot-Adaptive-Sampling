import numpy as np
import math
import random

# latitude_range: [40.4436987, 40.4436065]
# longitude_range: [-79.9429164, -79.9428501]
# latitude_range: [40.443710, 40.443776]
# longitude_range: [-79.94304, -79.94299]
# latitude_range: [40.443815, 40.443841]
# longitude_range: [-79.943051, -79.943059]
# 40.4437200367
# longitude: -79.9429980467

# latitude: 40.4436962533
# longitude: -79.9430444017

latitude1 = 40.4436052
longitude1 = -79.9429624

latitude2 = 40.4436346
longitude2  -79.9429142

min_lat = min(latitude1, latitude2)
max_lat = max(latitude1, latitude2)
min_lng = min(longitude1, longitude2)
max_lng = max(longitude1, longitude2)
resolution = 0.000005

print("latitude_range: ["+min_lat+", "+max_lat+"]")
print("longitude_range: ["+min_lng+", "+max_lng+"]")

gps_file = open("random_GPS.txt", "w")
temp_file = open("random_temperature.txt","w")

for lat in np.arange(min_lat, max_lat, resolution):
	for lng in np.arange(min_lng, max_lng, resolution):
		gps_file.write("%f , %f \n" %(lat,lng))
		temp_file.write("%f\n" % (5.90 + random.uniform(-1,1)))

gps_file.close()
temp_file.close()	