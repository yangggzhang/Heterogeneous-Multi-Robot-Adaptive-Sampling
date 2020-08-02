#!/bin/sh
git submodule update --init --recursive
sudo apt-get install python-catkin-tools python-pip
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic
pip install numpy scipy sklearn --user