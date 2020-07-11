#!/usr/bin/env python
import sys
import rospy
import time
import serial
import re
from collections import deque
import numpy as np
from sampling_msg.srv import RequestMeasurement, RequestMeasurementResponse

import itertools
import numpy as np
import matplotlib.pyplot as plt
import rospkg

class MeasurementSimulator(object):

    def __init__(self):
        self.position_x = None
        self.position_y = None
        self.measurement = None
        self.polyfit_order = None
        self.polyfit_coef = None
        self.measurement_simulation_server = None

    def polyfit2d(self, x, y, z, order=3):
        ncols = (order + 1)**2
        G = np.zeros((x.size, ncols))
        ij = itertools.product(range(order+1), range(order+1))
        for k, (i,j) in enumerate(ij):
            G[:,k] = x**i * y**j
        m, _, _, _ = np.linalg.lstsq(G, z)
        return m

    def polyval2d(self, x, y, m):
        order = int(np.sqrt(len(m))) - 1
        ij = itertools.product(range(order+1), range(order+1))
        z = 0.0
        for a, (i,j) in zip(m, ij):
            z += a * x**i * y**j
        return z
    
    def loadposition(self, measurement_trial):
        position_x = []
        position_y = []
        position_file = rospack.get_path('sampling_data') + "/data/position/" + measurement_trial + ".txt"
        with open(position_file, "r") as filestream:
            for line in filestream:
                new_x, new_y = currentline = line.split(",")
                position_x.append(float(new_x))
                position_y.append(float(new_y))
        return position_x, position_y

    def loadmeasurement(self, meansurement_trail):
        measurement_file = rospack.get_path('sampling_data') + "/data/measurement/" + measurement_trial + ".txt"
        return np.loadtxt(measurement_file)
    
    def simulatemeasurement(self, req):
        simulated_measurement = self.polyval2d(req.position.x, req.position.y, self.polyfit_coef)
        return RequestMeasurementResponse(simulated_measurement)

    def main(self):
        rospy.init_node('measurement_simulation_node')
        measurement_trial = rospy.get_param("~measurement_trial")
        self.polyfit_order = rospy.get_param('~poly_order', 5)
        self.position_x, self.position_y = self.loadposition(measurement_trial)
        self.measurement = self.loadmeasurement(measurement_trial)
        self.polyfit_coef = self.polyfit2d(self.position_x, self.position_y, self.measurement, order=self.polyfit_order)
        self.measurement_simulation_server = rospy.Service('measurement_simulation', RequestMeasurement, self.simulatemeasurement)
        rospy.spin()
        return 0

if __name__ == "__main__":
    temper = Temper()
    sys.exit(temper.main())
