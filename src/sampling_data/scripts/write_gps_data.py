#!/usr/bin/env python
import rospy
from sampling_msgs.msg import measurement
import numpy as np
import rospkg

rospack = rospkg.RosPack()

latitude = []
longitude = []
wifi = []

gt_resolution = 0.000002

def callback(data):
    data_path = rospack.get_path('sampling_data') + "/data/"
    wifi_gps_file = open(data_path + "wifi_gps2.txt", "w")
    rssi_file = open(data_path + "wifi_measurement2.txt","w")
    gt_gps_file = open(data_path + "wifi_gps_gt.txt", "w")
    latitude.append(data.latitude)
    longitude.append(data.longitude)
    wifi.append(data.measurement)
    for lat, lng, rssi in zip(latitude, longitude, wifi):
        wifi_gps_file.write("%f,%f \n" %(lat, lng))
        rssi_file.write("%f\n" % (rssi))
    wifi_gps_file.close()
    rssi_file.close()
    for lat in np.arange(min(latitude), max(latitude) + gt_resolution, gt_resolution):
            for lng in np.arange(min(longitude), max(longitude) + gt_resolution, gt_resolution):
                gt_gps_file.write("%f,%f \n" %(lat, lng))
    gt_gps_file.close()
    print 'Latitude', min(latitude), '-', max(latitude), 'Longitude : ', min(longitude) , ' - ', max(longitude)    
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/wifi_sample", measurement, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()