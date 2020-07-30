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
bag_name = "./hetero/"
obstacle_id = "obs_1_"
# obstacle_yaml = rospack.get_path('sampling_agent')+"/config/fake_agent_config.yaml"



def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()


for i in range(15):
	# for scenario in range(1,4):
	for scenario in range(1,2):
		rospy.init_node('en_Mapping', anonymous=True)
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)

		command = "rosbag record -O "+bag_name+"scenario"+str(scenario)+"_"+str(i)+".bag /sampling_performance"
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
		os.system("killall -9 gzserver")

		terminate_process_and_children(rosbag_proc)
		command = "mv "+bag_name+"scenario"+str(scenario)+"_"+str(i)+".bag.active "+bag_name+"scenario"+str(scenario)+"_"+str(i)+".bag"
		command = shlex.split(command)
		post_proce = subprocess.Popen(command)
		command = "rosbag reindex "+bag_name+"scenario"+str(scenario)+"_"+str(i)+".bag"
		command = shlex.split(command)
		post_proce = subprocess.Popen(command)