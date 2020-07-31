import roslaunch
import rospy
import rospkg
import yaml
import numpy
import random
import subprocess, shlex
import subprocess, os, signal
rospack = rospkg.RosPack()
from std_srvs.srv import Empty
import time
from roslaunch.parent import ROSLaunchParent

experiment_type = "hetero"  #hetero
bag_folder = "./" + experiment_type + "/"
experiment_num = 15 # num of trials to play in each scenario
task_time = 400
scenario_num = 3

if __name__ == '__main__':
	if not os.path.exists(bag_folder):
		os.makedirs(bag_folder)
		
	# set up node and launch config
	for scenario in range(1,scenario_num+1): # scenario is 1 indexed
		for i in range(experiment_num):
			parent = ROSLaunchParent("experiment", [], is_core=True)     # run_id can be any string
			parent.start()
			rospy.sleep(10)
			rospy.init_node('auto_experiment', anonymous=True)
			uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
			roslaunch.configure_logging(uuid)
			# launch gazebo simulation with corresponding world
			cli_args = [rospack.get_path('sampling_gazebo_simulation')+"/launch/two_ugv_one_uav_simulation.launch","scenario:="+str(scenario)]
			roslaunch_args = cli_args[1:]
			roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
			launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
			launch.start()
			# wait for gazebo 
			rospy.sleep(10)
			# record bag
			command = "rosbag record -O "+bag_folder+"scenario"+str(scenario)+"_"+str(i)+".bag /sampling_performance __name:=record_bag"
			command = shlex.split(command)
			rosbag_proc = subprocess.Popen(command)
			# launch sampling core
			launch2 = roslaunch.parent.ROSLaunchParent(uuid, 
			[(rospack.get_path('sampling_core')+"/launch/"+experiment_type+"geneous_adaptive_sampling.launch", "scenario:="+str(scenario))])
			launch2.start()
			rospy.loginfo("started")
			# wait for task_time (s)
			rospy.sleep(task_time)
			# shutdown sampling core
			launch2.shutdown()
			# finish bag recording
			command = "rosnode kill /record_bag"
			command = shlex.split(command)
			post_proce = subprocess.Popen(command)
			#shutdown gazebo simulation
			launch.shutdown()
			command = "killall -9 gzserver"
			command = shlex.split(command)
			post_proce = subprocess.Popen(command)
			time.sleep(10)
			parent.shutdown()
