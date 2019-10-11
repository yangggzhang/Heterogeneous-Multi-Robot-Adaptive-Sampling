#!/bin/sh
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-costmap-2d ros-kinetic-grid-map-ros ros-kinetic-ros-localization ros-kinetic-move-base*
sudo apt-get install ros-melodic-octomap ros-melodic-octomap-msgs ros-melodic-costmap-2d ros-melodic-grid-map-ros ros-melodic-ros-localization ros-melodic-move-base*
git submodule update --init --recursive
cd src/sampling_core/libgp/
mkdir build
cd build
cmake ..
make
pip install pyspread openpyxl scipy cvxopt scandir --user
