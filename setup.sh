#!/bin/sh
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libignition-math3 libsdformat5 gazebo8 libgazebo8* ros-kinetic-gazebo8-* ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-costmap-2d ros-kinetic-costmap-2d
git submodule update --init --recursive
cd src/swarm-primitive-dp/
mkdir generated_launch
cd swarm_simulator/src/cmuswarm_swarm/libgp
mkdir build
cd build
cmake ..
make
cd ../../../../../../..
pip install pyspread openpyxl scipy cvxopt --user
