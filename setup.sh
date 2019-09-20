#!/bin/sh
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-costmap-2d ros-kinetic-costmap-2d ros-kinetic-grid-map-ros
git submodule update --init --recursive
cd src/simulation/swarm-primitive-dp
mkdir generated_launch
cd swarm_simulator/src/cmuswarm_swarm/libgp/
mkdir build
cd build
cmake ..
make
cd ../../../../../../..
cd sampling_core/libgp/
mkdir build
cd build
cmake ..
make
pip install pyspread openpyxl scipy cvxopt scandir --user
