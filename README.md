# Introduction
This is ROS workspace for CMU MRSD 2018 Team G's capstone project - Hetegeneous Multi-Robot Sampling. <br />
The detailed descriptions for this project can be found via https://mrsdprojects.ri.cmu.edu/2018teamg/
# Installation
To install all required ROS packages, dependencies and python packages. Please run the following command in terminal first.
```bat
./setup.sh
```
User also needs to make two modifications to the swarm-primitive-dp submodule to make the code base built.
- Modification 1
  - Open **swarm-primitive-dp/swarm_simulator/src/cmuswarm_swarm/package.xml** using your favorite editor.
  - Modify line 50 from **<exec_depend>message_runtime</exec_depend>** to **<run_depend>message_runtime</run_depend>**
- Modification 2
  - Open **swarm-primitive-dp/swarm_simulator/src/cmuswarm_swarm/CMakeLists.txt** using your favorite editor.
  - Comment out lines from 60 to 62.
  ```bat
  #  add_message_files(
  #    FILES
  #  )
  ```
Add following commands to the end of **~/.bashrc**. Please modify **${path-to-heterogeneous-sampling}** according to the actual path.
```bat
source ${path-to-heterogeneous-sampling}/src/swarm-primitive-dp/swarm_simulator/devel/setup.sh
alias killgazebo='killall -9 gazebo & killall -9 gzserver & killall -9 gzclient'
alias killros='killall -9 roscore & killall -9 rosmaster'
alias killswarm="pkill -f swarm"
# Swarm primitives workspace
export SWARM_WS=${path-to-heterogeneous-sampling}/src/swarm-primitive-dp
alias swarm_ck='export ck_dir=`pwd`; cd ${path-to-heterogeneous-sampling}/src/swarm-primitive-dp/swarm_simulator; catkin_make; cd $ck_dir'
alias swarm_run='export ck_dir=`pwd`; cd ${path-to-heterogeneous-sampling}/src/swarm-primitive-dp/scripts; ./run.sh; cd $ck_dir'
```
Please also modify the path on line 14 and 41 in  <br /> 
**swarm-primitive-dp/swarm_simulator/src/cmuswarm_swarm/include/load_data.h**  <br />
to the actual file path, etc  <br />
**${path-to-heterogeneous-sampling}/src/swarm-primitive-dp/swarm_simulator/src/cmuswarm_swarm/include/*.txt**
# Run the project
```bat
swarm_run
```
