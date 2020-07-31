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

# bag_folder = "./homo/"
bag_folder = "./hetero/"
experiment_num = 5 # num of trials to play in each scenario
task_time = 1000
scenario_num = 15

def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
        print(parent)
    except psutil.NoSuchProcess:
        print("parent process not existing")
        return
    children = parent.children(recursive=True)
    print(children)
    for process in children:
        print("try to kill child: " + str(process))
        process.send_signal(sig)

class Roscore(object):
    """
    roscore wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    """
    __initialized = False
    def __init__(self):
        if Roscore.__initialized:
            raise Exception("You can't create more than 1 instance of Roscore.")
        Roscore.__initialized = True
    def run(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
            self.roscore_pid = self.roscore_process.pid  # pid of the roscore process (which has child processes)
        except OSError as e:
            sys.stderr.write('roscore could not be run')
            raise e
    def terminate(self):
        print("try to kill child pids of roscore pid: " + str(self.roscore_pid))
        kill_child_processes(self.roscore_pid)
        self.roscore_process.terminate()
        self.roscore_process.wait()  # important to prevent from zombie process
        Roscore.__initialized = False

if __name__ == '__main__':
	roscore = Roscore()

	if not os.path.exists(bag_folder):
		os.makedirs(bag_folder)

	for scenario in range(1,scenario_num+1): # scenario is 1 indexed

		for i in range(experiment_num):
			# set up node and launch config
			roscore.run()
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
			rospy.sleep(2.5)
			
			# record bag
			command = "rosbag record -O "+bag_folder+"scenario"+str(scenario)+"_"+str(i)+".bag /sampling_performance __name:=record_bag"
			command = shlex.split(command)
			rosbag_proc = subprocess.Popen(command)
			
			# launch sampling core
			launch2 = roslaunch.parent.ROSLaunchParent(uuid, 
			[(rospack.get_path('sampling_core')+"/launch/heterogeneous_adaptive_sampling.launch", "scenario:="+str(scenario))])
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
			rospy.sleep(30.0)
			roscore.terminate()