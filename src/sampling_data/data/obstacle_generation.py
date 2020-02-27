import rospy
import numpy as np

collision_radius = 10
# obstacle_pos: [4,3,
#                6,6,
#                8,9]
obstacle_pos= [1,2]
# data = rosparam.load_file("fake_agent_config.yaml")
# collision_radius = rospy.get_param("collision_radius")
# obstacle_pos = rospy.get_param("obstacle_pos")
obstacle_pos = np.array(obstacle_pos).reshape(-1,2)


latitude1 = 0
longitude1 = 0

latitude2 = 50
longitude2 = 50

min_lat = min(latitude1, latitude2)
max_lat = max(latitude1, latitude2)
min_lng = min(longitude1, longitude2)
max_lng = max(longitude1, longitude2)
resolution = 1

gps_file = open("obstacle_pos.txt", "w")


def distance(lat1, lng1, lat2, lng2):
	d_lat = lat1 - lat2
	d_lng = lng1 - lng2
	return np.sqrt((d_lat * d_lat) + (d_lng * d_lng))

for lat in np.arange(min_lat, max_lat + resolution, resolution):
    for lng in np.arange(min_lng, max_lng + resolution, resolution):
        for i in range(obstacle_pos.shape[0]):
            obs_lat = obstacle_pos[i,0]
            obs_long = obstacle_pos[i,1]
            distance = np.sqrt((lat-obs_lat)**2+(lng-obs_long)**2)
            if distance<collision_radius:
                gps_file.write("%f,%f \n" %(lat, lng))
gps_file.close()





