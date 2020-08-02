[![ROS Distro: Melodic](https://img.shields.io/badge/ROS-Melodic-blue.svg)](http://wiki.ros.org/melodic)
[![License: BSD](https://img.shields.io/badge/License-BSD-yellow.svg)](./LICENSE)

We are going to present our paper at IROS 2020. The official paper is comming soon!

# Adaptive Informative Sampling with Environment Partitioning for Heterogeneous Multi-Robot Systems
Multi-robot systems are widely used in environmental exploration and modeling, especially in hazardous environments. However, different types of robots are limited by different mobility, battery life, sensor type, etc. Heterogeneous robot systems are able to utilize various types of robots and provide solutions where robots are able to compensate each other with their different capabilities. In this paper, we consider the problem of sampling and modeling environmental characteristics with a heterogeneous team of robots. To utilize heterogeneity of the system while remaining computationally tractable, we propose an environmental partitioning approach that leverages various robot capabilities by forming a uniformly defined heterogeneity cost space. We combine with the mixture of Gaussian Processes model-learning framework to adaptively sample and model the environment in an efficient and scalable manner. We demonstrate our algorithm in field experiments with ground and aerial vehicles.

# Installation
First install **Full-Desktop** ROS Melodic by following the instructions : http://wiki.ros.org/melodic/Installation/Ubuntu. <br />
To install all additional ROS packages, dependencies and python packages. Please run the following command in terminal first.
```bat
./setup.sh
```
# Build the project
```
catkin build
```
# Run the project
```bat
roslaunch sampling_core centralized_sampling.launch
```
