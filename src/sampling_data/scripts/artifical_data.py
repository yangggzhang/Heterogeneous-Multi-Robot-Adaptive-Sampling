#!/usr/bin/env python
import sys
import rospy
import time
import numpy as np
import rospkg
import random

rospack = rospkg.RosPack()

class ArtificalData(object):
    def __init__(self, name = "data_generation_node"):
        rospy.init_node(name)
        self.latitude_range = []
        self.longitude_range = []
        self.resolution = 1.0
        self.scale= 1.0
        self.temperature_file = ""
        self.location_file = ""
        self.heatsource_latitude = []
        self.heatsource_longitude = []
        self.heatsource_temperature = []
        self.heatsource_range = []
        self.ambient_temp = []
        self.min_lat = 0.0
        self.max_lat = 0.0
        self.min_lng = 0.0
        self.max_lng = 0.0
        self.data_path = ""

    def load_data(self):
        self.latitude_range = rospy.get_param("~latitude_range")
        self.longitude_range = rospy.get_param("~longitude_range")
        self.resolution = rospy.get_param("~resolution")
        self.scale = rospy.get_param("~scale")
        self.temperature_file = rospy.get_param("~temperature_file_name")
        self.location_file = rospy.get_param("~location_file_name")
        self.heatsource_latitude = rospy.get_param("~heatsource_latitude")
        self.heatsource_longitude = rospy.get_param("~heatsource_longitude")
        self.heatsource_temperature = rospy.get_param("~heatsource_temperature")
        self.heatsource_range = rospy.get_param("~heatsource_range")
        self.noise = rospy.get_param("~noise")
        self.ambient_temp = rospy.get_param("~ambient_temp")
        self.min_lat = min(self.latitude_range)
        self.max_lat = max(self.latitude_range)
        self.min_lng = min(self.longitude_range)
        self.max_lng = max(self.longitude_range)     
        self.data_path = rospack.get_path('sampling_data') + "/data/"


    def distance(self, lat1, lng1, lat2, lng2):
        d_lat = lat1 - lat2
        d_lng = lng1 - lng2
        return np.sqrt(d_lat * d_lat + d_lng * d_lng)
    
    def write(self):      
        gps_file = open(self.data_path + self.location_file, "w")
        temp_file = open(self.data_path + self.temperature_file,"w")
        for lat in np.arange(self.min_lat, self.max_lat + self.resolution, self.resolution):
            for lng in np.arange(self.min_lng, self.max_lng + self.resolution, self.resolution):
                temperature = self.ambient_temp + random.uniform(-self.noise, self.noise)
                for heat_lat, heat_lng, max_heat, max_range in zip(self.heatsource_latitude, self.heatsource_longitude, self.heatsource_temperature, self.heatsource_range):
                    d = self.distance(lat, lng, heat_lat, heat_lng)
                    if d <= max_range:
                        temperature = temperature + (max_range - d)/max_range * (max_heat - self.ambient_temp)
                gps_file.write("%f,%f \n" %(lat * self.scale, lng * self.scale))
                temp_file.write("%f\n" % (temperature))
        gps_file.close()
        temp_file.close()
        print "Finish data generation!"
        
if __name__ == "__main__":
    data = ArtificalData()
    data.load_data()
    data.write()
