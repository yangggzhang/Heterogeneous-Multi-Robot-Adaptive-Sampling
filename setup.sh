#!/bin/sh
sudo apt-get update
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-costmap-2d ros-kinetic-costmap-2d ros-kinetic-grid-map-ros
git submodule update --init --recursive
cd src/simulation/swarm-primitive-dp
mkdir generated_launch
cd ../sampling-core/libgp
mkdir build
cd build
cmake ..
make
<<<<<<< HEAD
cd ../../../../..
=======
cd ../../../../../..
>>>>>>> 9252d6b89ddf8292c473e2b7fcf826d4f22b0687
pip install pyspread openpyxl scipy cvxopt --user
