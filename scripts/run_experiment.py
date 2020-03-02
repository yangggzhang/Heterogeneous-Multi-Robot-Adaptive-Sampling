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
# obstacle_yaml = rospack.get_path('robot_agent')+"/config/fake_agent_config.yaml"


def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()



def call_generate_obstacle():
	command = "python "+rospack.get_path('sampling_data')+"/data/obstacle_generation.py"
	command = shlex.split(command)
	post_proce = subprocess.Popen(command)


for i in range(15):
	rospy.init_node('en_Mapping', anonymous=True)
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	call_generate_obstacle()

	command = "rosbag record -O "+bag_name+obstacle_id+str(i)+".bag /report /agent_location_channel /sampling_visualization"
	command = shlex.split(command)
	rosbag_proc = subprocess.Popen(command)

	launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/yunfei/heterogeneous-sampling/src/sampling_core/launch/sampling_simulation.launch"])
	launch2.start()


	launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/yunfei/heterogeneous-sampling/src/robot_agent/launch/multi_fake_agent.launch"])
	launch.start()

	rospy.loginfo("started")

	rospy.sleep(200)
	# 2 seconds later
	launch.shutdown()
	launch2.shutdown()
	terminate_process_and_children(rosbag_proc)
	command = "mv "+bag_name+obstacle_id+str(i)+".bag.active "+bag_name+obstacle_id+str(i)+".bag"
	command = shlex.split(command)
	post_proce = subprocess.Popen(command)
	command = "rosbag reindex "+bag_name+obstacle_id+str(i)+".bag"
	command = shlex.split(command)
	post_proce = subprocess.Popen(command)