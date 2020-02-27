import numpy as np
import math
import random
from scipy.stats import multivariate_normal

latitude1 = 0
longitude1 = 0

latitude2 = 50
longitude2 = 50
scale = 1

min_lat = min(latitude1, latitude2)
max_lat = max(latitude1, latitude2)
min_lng = min(longitude1, longitude2)
max_lng = max(longitude1, longitude2)
resolution = 1

print("latitude_range: [" + str(min_lat) + ", " + str(max_lat) + "]")
print("longitude_range: [" + str(min_lng) + ", " + str(max_lng) + "]")

gps_file = open("gt_fake_GPS.txt", "w")
temp_file = open("gt_fake_temperature.txt","w")

# # gps_file = open("gt_GPS.txt", "w")
# # temp_file = open("gt_temperature.txt","w")

# heat_source_lat = [40.000010, 40.000030]
# heat_source_lng = [-79.000010, -79.000030]
# heat_temp = [ 15.0, 30.0]

# ambient_temp = 5.0

# max_distance = 0.00001


ground_truth_mu = [0, 0, 
                  74, 10, 
                  14, 8]

ground_truth_sig = [5, 3, 
                   12, 8, 
                   4, 8]


gaussian_weights= [200,140,1500]

ground_truth_mu = np.array(ground_truth_mu).reshape(-1,2)
ground_truth_sig = np.array(ground_truth_sig).reshape(-1,2)
print(ground_truth_mu)
print(ground_truth_sig)



# # def distance(lat1, lng1, lat2, lng2):
# # 	d_lat = lat1 - lat2
# # 	d_lng = lng1 - lng2
# # 	return np.sqrt(d_lat * d_lat + d_lng * d_lng)

# pos = []
for lat in np.arange(min_lat, max_lat + resolution, resolution):
    for lng in np.arange(min_lng, max_lng + resolution, resolution):
        temperature = 0
        for m in range(ground_truth_mu.shape[0]):
            y = multivariate_normal.pdf(np.array([lat,lng]), ground_truth_mu[m,:], ground_truth_sig[m,:]*np.eye(2))
            # print(y)
            temperature+=gaussian_weights[m]*y+random.uniform(-2, 2)
        gps_file.write("%f,%f \n" %(lat * scale, lng * scale))
        temp_file.write("%f\n" % (temperature))

gps_file.close()
temp_file.close()	