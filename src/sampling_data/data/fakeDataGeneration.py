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
ground_truth_type = rospy.get_param('ground_truth_type')
poly_coeff = rospy.get_param('poly_coeff')

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

init_gps_file = open("init_final_fake_GPS.txt", "w")
init_temp_file = open("init_final_fake_temperature.txt","w")

gt_gps_file = open("gt_final_fake_GPS.txt", "w")
gt_temp_file = open("gt_final_fake_temperature.txt","w")

ground_truth_mu = np.array(ground_truth_mu).reshape(-1,2)
ground_truth_sig = np.array(ground_truth_sig).reshape(-1,2)
print(ground_truth_mu)
print(ground_truth_sig)

def poly(x, y):
    temperature = poly_coeff[0] + poly_coeff[1]*x + poly_coeff[2]*y + poly_coeff[3]*(x**2) + \
    poly_coeff[4]*x*y + poly_coeff[5]*(y**2) + poly_coeff[6]*(x**3) + \
    poly_coeff[7]*(x**2)*y + poly_coeff[8]*x*(y**2) + poly_coeff[9]*(y**3) + poly_coeff[10]*x**4 + poly_coeff[11]*((x**3))*y + \
    poly_coeff[12]*(x**2)*(y**2) + poly_coeff[13]*x*(y**3) + poly_coeff[14]*(y**4) + poly_coeff[15]*(x**5) + poly_coeff[16]*(x**4)*y + \
    poly_coeff[17]*(x**3)*(y**2) + poly_coeff[18]*(x**2)*(y**3) + poly_coeff[19]*x*(y**4) + poly_coeff[20]*y**5
    temperature = max(temperature*1.5, 0)
    return temperature

for lat in np.arange(min_lat, max_lat, init_resolution):
    for lng in np.arange(min_lng, max_lng, init_resolution):
        temperature = 0
        if ground_truth_type == 0:
            for m in range(ground_truth_mu.shape[0]):
                y = multivariate_normal.pdf(np.array([lat,lng]), ground_truth_mu[m,:], ground_truth_sig[m,:]*np.eye(2))
                temperature+=gaussian_weights[m]*y
                # print(y)
        elif ground_truth_type == 1:
            temperature = poly(lat, lng)
        temperature+=np.random.normal(0, noise_sigma)
        init_gps_file.write("%f,%f \n" %(lat , lng ))
        init_temp_file.write("%f\n" % (temperature))

init_gps_file.close()
init_temp_file.close()

for lat in np.arange(min_lat, max_lat, resolution):
    for lng in np.arange(min_lng, max_lng, resolution):
        temperature = 0
        if ground_truth_type == 0:
            for m in range(ground_truth_mu.shape[0]):
                y = multivariate_normal.pdf(np.array([lat,lng]), ground_truth_mu[m,:], ground_truth_sig[m,:]*np.eye(2))
                temperature+=gaussian_weights[m]*y
                # print(y)
        elif ground_truth_type == 1:
            temperature = poly(lat, lng)
        # temperature+=np.random.normal(0, noise_sigma)
        gt_gps_file.write("%f,%f \n" %(lat , lng))
        gt_temp_file.write("%f\n" % (temperature))

gt_gps_file.close()
gt_temp_file.close()	