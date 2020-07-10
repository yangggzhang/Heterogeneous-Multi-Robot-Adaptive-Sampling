# Introduction
Multi-robot   systems   are   widely   used   in   envi-ronmental  exploration  and  modeling,  especially  in  hazardousenvironments. However, different types of robots are limited bydifferent mobility, battery life, sensor type, etc. Heterogeneousrobot   systems   are   able   to   utilize   various   types   of   robotsand  provide  solutions  where  robots  are  able  to  compensateeach  other  with  their  different  capabilities.  In  this  paper,  weconsider the problem of sampling and modeling environmentalcharacteristics  with  a  heterogeneous  team  of  robots.  To  utilizeheterogeneity  of  the  system  while  remaining  computationallytractable,  we  propose  an  environmental  partitioning  approachthat leverages various robot capabilities by forming a uniformlydefined heterogeneity cost space. We combine with the mixtureof Gaussian Processes model-learning framework to adaptivelysample and model the environment in an efficient and scalablemanner.  We  demonstrate  our  algorithm  in  field  experimentswith  ground  and  aerial  vehicles.

# Installation
First install **Full-Desktop** ROS Kinetic by following the instructions : http://wiki.ros.org/kinetic/Installation/Ubuntu. <br />
To install all additional ROS packages, dependencies and python packages. Please run the following command in terminal first.
```bat
./setup.sh
```
# Build the project
Add following commands to the end of **~/.bashrc**. Please modify **${path-to-heterogeneous-sampling}** according to the actual path.
```
catkin build
```
# Run the project
```bat
roslaunch sampling_core centralized_sampling.launch
```
