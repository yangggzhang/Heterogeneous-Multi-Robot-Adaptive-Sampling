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

# latitude: 40.4436052
# longitude: -79.9429624


# latitude: 40.4436346
# longitude: -79.9429142



min_lat = 40.4436052
max_lat = 40.4436346
min_lng = -79.9429624
max_lng = -79.9429142
resolution = 0.000005

gps_file = open("random_GPS.txt", "w")
temp_file = open("random_temperature.txt","w")

for lat in np.arange(min_lat, max_lat, resolution):
	for lng in np.arange(min_lng, max_lng, resolution):
		gps_file.write("%f , %f \n" %(lat,lng))
		temp_file.write("%f\n" % (5.90 + random.uniform(-1,1)))

gps_file.close()
temp_file.close()	