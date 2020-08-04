[![ROS Distro: Melodic](https://img.shields.io/badge/ROS-Melodic-blue.svg)](http://wiki.ros.org/melodic)
[![License: BSD](https://img.shields.io/badge/License-BSD-yellow.svg)](./LICENSE) <br/>

# Adaptive Informative Sampling with Environment Partitioning for Heterogeneous Multi-Robot Systems #
by Yunfei Shi ((Linkedin)[https://www.linkedin.com/in/shi-yunfei/]/(Google Scholar)[https://scholar.google.com/citations?user=lU47d44AAAAJ&hl=en]), [Ning Wang*](https://www.linkedin.com/in/ning-wang-cmu/), [Jianmin Zheng*](https://www.linkedin.com/in/jianmimzheng/), [Yang Zhang*](https://www.linkedin.com/in/yang-zhang-cmu/), [Sha Yi](https://www.ri.cmu.edu/ri-people/yisha-sha-yi/), [Wenhao Luo](http://www.contrib.andrew.cmu.edu/~wenhaol/), and [Katia Sycara](https://www.ri.cmu.edu/ri-faculty/katia-sycara/) <br/>
IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2020. <br/>
The final submission version [pdf](docs/AdaptiveInformativeSamplingwithEnvironmentPartitioningforHeterogeneousMulti-RobotSystems.pdf), and also video [here](https://drive.google.com/file/d/1uPgnyV66UEWomSi_KKH2vZLEvRN9gDTj/view?usp=sharing).

## Introduction ##
Multi-robot systems are widely used in environmental exploration and modeling, especially in hazardous environments. However, different types of robots are limited by different mobility, battery life, sensor type, etc. Heterogeneous robot systems are able to utilize various types of robots and provide solutions where robots are able to compensate each other with their different capabilities. In this paper, we consider the problem of sampling and modeling environmental characteristics with a heterogeneous team of robots. To utilize heterogeneity of the system while remaining computationally tractable, we propose an environmental partitioning approach that leverages various robot capabilities by forming a uniformly defined heterogeneity cost space. We combine with the mixture of Gaussian Processes model-learning framework to adaptively sample and model the environment in an efficient and scalable manner. We demonstrate our algorithm in field experiments with ground and aerial vehicles.

## Support Platform ##
we present a system architecture  for  heterogeneous  multi-robot  informative  samplingwith a modularized design that allows for flexible scale-ups and extensions  in  both  robot  characteristics  and  team  size. The current robot platform we support includes:

| Robot Platform | Description | Simulation | Physical Platform|
| :-: | :-: | :-: | :-: |
|[Clearpath Jackal UGV](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/#:~:text=UNMANNED%20GROUND%20VEHICLE,%2Dthe%2Dbox%20autonomous%20capability) | <img src="docs/jackal.jpeg?raw=true" height="200"> |:heavy_check_mark:|:heavy_check_mark:|
|[Asctec Pelican UAV](https://mrsdprojects.ri.cmu.edu/2018teamg/documentation/asctec-pelican-uav-setup-guidance/) |<img src="docs/pelican.jpg?raw=true" height="200">|:interrobang:|:heavy_check_mark:|
|[Hector Quadrator](http://wiki.ros.org/hector_quadrotor) | <img src="docs/hector.png?raw=true" height="200"> |:heavy_check_mark:|:interrobang:|

Check [sampling_agent](src/sampling_agent/include/sampling_agent/sampling_agent.h) to add your own robot to this sampling framework.

## Installation ##
First install **Full-Desktop** ROS Melodic by the [instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). <br />
To install all additional ROS packages, dependencies and python packages. Please run the setup script.
```bat
./setup.sh
```
## Build the package ##
This work depends on multi ROS pacakgaes for robot simulation/deployment, it may take up to 5 minutes to build ths project.
```
catkin build
```
## Run the simulation ##
Don't forget to `source devel/setup.bash` before launch any files. <br/>
First, launch the simulation environment in Gazebo in one terminal. Check or add customize simulation environments [here](src/sampling_gazebo_simulation/worlds).
```bat
roslaunch sampling_gazebo_simulation two_ugv_one_uav_simulation.launch
```
Second, launch the heterogeneous multi-robot adaptive sampling algorithm in the second terminal. Please make sure the agent information fed to the sampling algorithim is consistent with the gazebo simulation by checking the [ros parameters](src/sampling_core/launch/heterogeneous_adaptive_sampling.launch). Users can also adjust or add heterogeneous primitives according to their robot systems [here](src/sampling_core/config) <br/>
```bat
roslaunch sampling_core heterogeneous_adaptive_sampling.launch
```
You can also directly monitor the sampling performance by listenting to the `/sampling_performance` channel, which includes the number of samples collected, root mean square error for prediction, and average variance from prediction.<br />
```
rostopic echo /sampling_performance
```

<br />
<img src="docs/heterogeneous_sampling_simulation.gif?raw=true">
The left window is the simulation running in Gazebo. The right one is the visualization of heterogeneous multi-robot adaptive sampling in rviz. The leftmost grid is showing the heterogeneous environment partition and agents' locations, the middle one is showing the real-time model prediction, and the rightmost one is showing the real-time uncertainties from prediction.


## Results ##
The robots in our simulation are different in speed, battery life and traversability. 

### Environment Partition using Heterogeneity Primitives ###
<img src="docs/partition.png?raw=true">
The red, blue and green dots represent the current locations of the ground robot 1, 2 and the aerial robot respectively. The corresponding shallow areas are their responsible regions. The black circle denotes an obstacle that the robot 1 and 2 need to avoid. The first partition from left uses normal Voronoi Diagram; the second adds speed heterogeneity; the third adds battery life, and the last adds traversability.

### Heterogeneous Informative Sampling ###
<img src="docs/result_fig4.png?raw=true">
Informative sampling performance comparison between heterogeneous and homogeneous multi-robot sampling algorithms. We run each algorithm on the same dataset 45 times with random robot initial locations and obstacle positions. The shallow areas represent the variance range. The red vertical line indicates the time when the aerial robot stopped operation.

## Citation ##
If you find this work or code helpful, please cite:
```
@InProceedings{hetero_sampling,
author = {Yunfei Shi, Ning Wang, Jianmin Zheng, Yang Zhang, Sha Yi, Wenhao Luo, and Katia Sycara},
title = {Adaptive Informative Sampling with Environment Partitioning for Heterogeneous Multi-Robot Systems},
booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
month = {Oct},
year = {2020}
}
```

## Acknowledgement ##
This work is primarity from [Team SAMP](https://mrsdprojects.ri.cmu.edu/2018teamg/)'s (Yunfei Shi, Ning Wang, Jianmin Zheng and Yang Zhang) capstone project when they were pursing their masters' degrees in [Robotic Systems Development (MRSD)](https://mrsd.ri.cmu.edu/) at Carnegie Mellon University the Robotics Institute. Special thanks to [John M. Dolan](https://www.ri.cmu.edu/ri-faculty/john-m-dolan/), [Dimitrios (Dimi) Apostolopoulos](https://www.ri.cmu.edu/ri-faculty/dimitrios-dimi-apostolopoulos/) and [Sarah Conte](https://www.ri.cmu.edu/ri-people/sarah-conte/) for the support and advice.
