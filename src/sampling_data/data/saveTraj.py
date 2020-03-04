#!/usr/bin/env python
import rospy
from sampling_msgs.msg import measurement
import numpy as np
import rospkg

rospack = rospkg.RosPack()
data_path = rospack.get_path('sampling_data') + "/data/"

def callback(data):
	if data.robot_id == 0:
		with open(data_path+"robot_0_traj.txt", "a") as robo_0_traj_file:
			robo_0_traj_file.write("%f,%f \n" %(data.location_x, data.location_y))
	elif data.robot_id == 1:
		with open(data_path+"robot_1_traj.txt", "a") as robo_1_traj_file:
			robo_1_traj_file.write("%f,%f \n" %(data.location_x, data.location_y))
	else:
		pass
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sample_collection_channel", measurement, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	open(data_path+"robot_0_traj.txt", 'w').close()
	open(data_path+"robot_1_traj.txt", 'w').close()
	listener()