import roslaunch
import rospy
import rospkg
import yaml
import numpy
import random
import subprocess, shlex
import subprocess, os, signal
rospack = rospkg.RosPack()

# bag_name = "./homo/"
bag_folder = "./hetero/"

if not os.path.exists(bag_folder):
    os.makedirs(bag_folder)
for i in range(2):
	# for scenario in range(1,4):
	for scenario in range(1,2):
		rospy.init_node('en_Mapping', anonymous=True)
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)

		command = "rosbag record -O "+bag_folder+"scenario"+str(scenario)+"_"+str(i)+".bag /sampling_performance __name:=record_bag"
		command = shlex.split(command)
		rosbag_proc = subprocess.Popen(command)

		launch = roslaunch.parent.ROSLaunchParent(uuid, 
		[(rospack.get_path('sampling_gazebo_simulation')+"/launch/two_ugv_one_uav_simulation.launch","scenario:="+str(scenario))])
		launch.start()

		rospy.sleep(2)

		launch2 = roslaunch.parent.ROSLaunchParent(uuid, 
		[(rospack.get_path('sampling_core')+"/launch/heterogeneous_adaptive_sampling.launch", "scenario:="+str(scenario))])
		launch2.start()

		rospy.loginfo("started")

		rospy.sleep(200)
		# 2 seconds later
		launch.shutdown()
		launch2.shutdown()
		command = "killall -9 gzserver"
		command = shlex.split(command)
		post_proce = subprocess.Popen(command)
		command = "rosnode kill /record_bag"
		command = shlex.split(command)
		post_proce = subprocess.Popen(command)