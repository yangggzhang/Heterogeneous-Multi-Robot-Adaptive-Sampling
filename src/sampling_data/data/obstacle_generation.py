import rospy
import roslib
roslib.load_manifest("rosparam")
import rosparam
import rospkg
import numpy as np

rospack = rospkg.RosPack()
paramlist = rosparam.load_file(rospack.get_path('sampling_agent')+"/config/fake_agent_config.yaml")
for params, ns in paramlist:
    rosparam.upload_params(ns, params)
collision_radius = rospy.get_param('collision_radius')
obstacle_pos = rospy.get_param('obstacle_pos')
map_range = rospy.get_param('map_range')
resolution = rospy.get_param('map_resolution')
obstacle_pos = np.array(obstacle_pos).reshape(-1,2)


latitude1 = map_range[0]
longitude1 = map_range[1]
latitude2 = map_range[2]
longitude2 = map_range[3]

min_lat = min(latitude1, latitude2)
max_lat = max(latitude1, latitude2)
min_lng = min(longitude1, longitude2)
max_lng = max(longitude1, longitude2)

obstacle_file = open(rospack.get_path('sampling_data')+"/data/obstacle_1.txt", "w")

for lat in np.arange(min_lat, max_lat + resolution, resolution):
    for lng in np.arange(min_lng, max_lng + resolution, resolution):
        for i in range(obstacle_pos.shape[0]):
            obs_lat = obstacle_pos[i,0]
            obs_long = obstacle_pos[i,1]
            # Circle Obstacle
            # distance = np.sqrt((lat-obs_lat)**2+(lng-obs_long)**2)
            # if distance<collision_radius:
            #     obstacle_file.write("%f,%f \n" %(lat, lng))

            # Square Obstacle
            if lat < obs_lat+collision_radius and lat > obs_lat-collision_radius and lng < obs_long+collision_radius and lng > obs_long-collision_radius:
            	 obstacle_file.write("%f,%f \n" %(lat, lng))
            	 
obstacle_file.close()





