# Introduction
This is ROS workspace for CMU MRSD 2018 Team G's capstone project - Hetegeneous Multi-Robot Sampling. <br />
The detailed descriptions for this project can be found via https://mrsdprojects.ri.cmu.edu/2018teamg/

The work is implemented based on [Adaptive Sampling and Online Learning in Multi-Robot Sensor Coverage with Mixture of Gaussian Processes](http://www.contrib.andrew.cmu.edu/~wenhaol/publications/ICRA18_AdaSam_Coverage.pdf)

# Installation
First install **Full-Desktop** ROS Kinetic by following the instructions : http://wiki.ros.org/kinetic/Installation/Ubuntu. <br />
To install all additional ROS packages, dependencies and python packages. Please run the following command in terminal first.
```bat
./setup.sh
```
# Build the project
Add following commands to the end of **~/.bashrc**. Please modify **${path-to-heterogeneous-sampling}** according to the actual path.
```
catkin_make
```
# Run the project
```bat
roslaunch sampling_core centralized_sampling.launch
```
