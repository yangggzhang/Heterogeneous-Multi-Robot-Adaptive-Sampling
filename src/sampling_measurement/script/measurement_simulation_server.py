#!/usr/bin/env python
import itertools
import numpy as np
import rospkg
import rospy
from sampling_msgs.srv import RequestMeasurement, RequestMeasurementResponse

class MeasurementSimulator(object):
    def __init__(self):
        rospy.init_node('measurement_simulation_node')
        measurement_trial = rospy.get_param("~measurement_trial")
        self.rospack = rospkg.RosPack()
        self.polyfit_order = rospy.get_param('~poly_order', 5)
        self.noise_stdev = rospy.get_param('~noise_stdev', 0.5)
        self.position_x, self.position_y = self.loadposition(measurement_trial)
        self.measurement = self.loadmeasurement(measurement_trial)
        self.polyfit_coef = self.polyfit2d(self.position_x, self.position_y, self.measurement, order=self.polyfit_order)
        self.generateGroundTruth(measurement_trial)
        self.measurement_simulation_server = rospy.Service('measurement_simulation', RequestMeasurement, self.simulatemeasurement)
        rospy.spin()

    def polyfit2d(self, x, y, z, order=3):
        ncols = (order + 1)**2
        G = np.zeros((x.size, ncols))
        ij = itertools.product(range(order+1), range(order+1))
        for k, (i,j) in enumerate(ij):
            G[:,k] = x**i * y**j
        m, _, _, _ = np.linalg.lstsq(G, z, rcond=None)
        return m

    def polyval2d(self, x, y, m):
        order = int(np.sqrt(len(m))) - 1
        ij = itertools.product(range(order+1), range(order+1))
        z = 0.0
        for a, (i,j) in zip(m, ij):
            z += a * x**i * y**j
        return z
    
    def generateGroundTruth(self, measurement_trail):
        artifical_ground_truth = self.rospack.get_path('sampling_data') + "/measurement/artificial_" + measurement_trail + ".txt"
        data_file = open(artifical_ground_truth, "w")
        for x, y in zip(self.position_x, self.position_y):
            measurement = self.polyval2d(x, y, self.polyfit_coef)
            data_file.write("%f\n" %(measurement))
        data_file.close()

    def loadposition(self, measurement_trial):
        position_x = []
        position_y = []
        position_file = self.rospack.get_path('sampling_data') + "/location/" + measurement_trial + ".txt"
        with open(position_file, "r") as filestream:
            for line in filestream:
                new_x, new_y = line.split(",")
                position_x.append(float(new_x))
                position_y.append(float(new_y))
        return np.array(position_x), np.array(position_y)

    def loadmeasurement(self, measurement_trail):
        measurement_file = self.rospack.get_path('sampling_data') + "/measurement/" + measurement_trail + ".txt"
        return np.loadtxt(measurement_file)
    
    def simulatemeasurement(self, req):
        simulated_measurement = self.polyval2d(req.position.x, req.position.y, self.polyfit_coef)
        simulated_measurement += np.random.normal(0, self.noise_stdev)
        return RequestMeasurementResponse(simulated_measurement)

if __name__ == "__main__":
    measurement_simulation_server = MeasurementSimulator()
