#!/usr/bin/env python
import sys
# import rospy
import time
import numpy as np
# import rospkg
import random

MIN_X = 0
MAX_X = 10
MIN_Y = 0
MAX_Y = 10
RESOLUTION = 1

class MapGenerator(object):
    def __init__(self, min_x= MIN_X, max_x = MAX_X, min_y = MIN_Y, max_y = MAX_Y, resolution = RESOLUTION,file_name="test_map"):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.resolution = resolution
        self.file_name = file_name
    
    def generate_map(self):
        data_dir = "../map/" + self.file_name + ".txt"
        map_file = open(data_dir, "w")
        for x in np.arange(self.min_x, self.max_x + self.resolution, self.resolution):
            for y in np.arange(self.min_y, self.max_y + self.resolution, self.resolution):
                map_file.write("%f,%f \n" %(x, y))
        map_file.close()
        print("Map data generation!")
        
if __name__ == "__main__":
    generator = MapGenerator()
    generator.generate_map()