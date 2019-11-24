#!/usr/bin/env python
import sys
import rospy
import time
import serial
import re
from collections import deque
import numpy as np
from sampling_msgs.msg import temperature_measurement
from sampling_msgs.srv import RequestTemperatureMeasurement, RequestTemperatureMeasurementResponse

class Temper(object):

    def __init__(self):
        self.raw_temp_window = deque()
        self.fused_temp_window = deque()
        self.fused_temp = 0.0
        self.raw_temp = 0.0
        self.converged = False
        self.ser = []

    def read(self):
        try:
                line = self.ser.readline()
                m = re.search(r'[0-9.]+', line)
                if m is not None:
                    degC = (float(m.group(0)) - 32) / 1.8
                    # publish(degC, pub)
                    return degC
                  
        except KeyboardInterrupt:
                print('exiting')
                self.ser.close()
                return

    def is_converged(self, thre=0.1):
        # if len(self.fused_temp_window) >= 5:
        #     self.fused_temp_window.popleft()
        # self.fused_temp_window.append(self.fused_temp)
        temp_std = np.std(list(self.fused_temp_window))
        return (temp_std < thre)
        
    def collect_temperature_sample(self, req):
        while (not self.is_converged()):
          rospy.loginfo("Waiting for temperature measurement to converge")
        return RequestTemperatureMeasurementResponse(self.fused_temp)
        
    def main(self):
        # pub = rospy.Publisher('temp', Temperature, queue_size=1)
        converge_timer = time.time()
        rospy.init_node('node_temper')
        temperature_report_service_channel = rospy.get_param("~temperature_report_service_channel")
        temperature_report_service = rospy.Service(temperature_report_service_channel, RequestTemperatureMeasurement, self.collect_temperature_sample)
        temperature_publish_channel = rospy.get_param("~temperature_publish_channel")
        temperature_pub = rospy.Publisher(temperature_publish_channel, temperature_measurement, queue_size=10)
        usb_port = rospy.get_param("~USBPort")
        self.ser = serial.Serial(usb_port, 38400, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS )  # open serial port
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            degC = self.read()
            if not (degC is None):
                if len(self.raw_temp_window) >= 5:
                    self.raw_temp_window.popleft()
                self.raw_temp_window.append(degC)
                self.raw_temp = degC
            
            if len(self.raw_temp_window) > 0:
                self.fused_temp = np.mean(list(self.raw_temp_window))
                # check if converged
                now = time.time()
                self.fused_temp_window.append(self.fused_temp)
                if now - converge_timer > 0.2:
                    self.converged = self.is_converged()
                    converge_timer = time.time()
                    self.fused_temp_window.clear()
            msg = temperature_measurement()
            msg.header.stamp = rospy.Time.now()
            msg.raw_temperature = self.raw_temp
            msg.filtered_temperature = self.fused_temp
            msg.converged = self.converged
            temperature_pub.publish(msg)
            r.sleep()
        return 0

if __name__ == "__main__":
    temper = Temper()
    sys.exit(temper.main())
