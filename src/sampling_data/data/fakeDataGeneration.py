import rospy
import roslib
roslib.load_manifest("rosparam")
import rosparam
import rospkg
import numpy as np
import math
import random
from scipy.stats import multivariate_normal

rospack = rospkg.RosPack()
paramlist = rosparam.load_file(rospack.get_path('robot_agent')+"/config/fake_agent_config.yaml")
for params, ns in paramlist:
    rosparam.upload_params(ns, params)
ground_truth_mu = rospy.get_param('ground_truth_mu')
ground_truth_sig = rospy.get_param('ground_truth_sig')
map_range = rospy.get_param('map_range')
resolution = rospy.get_param('map_resolution')
init_resolution = rospy.get_param('init_resolution')
gaussian_weights = rospy.get_param('gaussian_weights')
noise_sigma = rospy.get_param('observation_noise_std')

latitude1 = map_range[0]
longitude1 = map_range[1]
latitude2 = map_range[2]
longitude2 = map_range[3]

min_lat = min(latitude1, latitude2)
max_lat = max(latitude1, latitude2)
min_lng = min(longitude1, longitude2)
max_lng = max(longitude1, longitude2)

print("latitude_range: [" + str(min_lat) + ", " + str(max_lat) + "]")
print("longitude_range: [" + str(min_lng) + ", " + str(max_lng) + "]")

init_gps_file = open("init_fake_GPS.txt", "w")
init_temp_file = open("init_fake_temperature.txt","w")

gt_gps_file = open("gt_fake_GPS.txt", "w")
gt_temp_file = open("gt_fake_temperature.txt","w")

ground_truth_mu = np.array(ground_truth_mu).reshape(-1,2)
ground_truth_sig = np.array(ground_truth_sig).reshape(-1,2)
print(ground_truth_mu)
print(ground_truth_sig)

for lat in np.arange(min_lat, max_lat + resolution, init_resolution):
    for lng in np.arange(min_lng, max_lng + resolution, init_resolution):
        temperature = 0
        for m in range(ground_truth_mu.shape[0]):
            y = multivariate_normal.pdf(np.array([lat,lng]), ground_truth_mu[m,:], ground_truth_sig[m,:]*np.eye(2))
            # print(y)
            temperature+=gaussian_weights[m]*y+np.random.normal(0, noise_sigma)
        init_gps_file.write("%f,%f \n" %(lat , lng ))
        init_temp_file.write("%f\n" % (temperature))

init_gps_file.close()
init_temp_file.close()

for lat in np.arange(min_lat, max_lat + resolution, resolution):
    for lng in np.arange(min_lng, max_lng + resolution, resolution):
        temperature = 0
        for m in range(ground_truth_mu.shape[0]):
            y = multivariate_normal.pdf(np.array([lat,lng]), ground_truth_mu[m,:], ground_truth_sig[m,:]*np.eye(2))
            # print(y)
            temperature+=gaussian_weights[m]*y+np.random.normal(0, noise_sigma)
        gt_gps_file.write("%f,%f \n" %(lat , lng))
        gt_temp_file.write("%f\n" % (temperature))

gt_gps_file.close()
gt_temp_file.close()	