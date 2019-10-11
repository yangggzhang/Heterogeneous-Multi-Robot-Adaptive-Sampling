#!/bin/bash
catkin_make --pkg sampling_msgs
catkin_make
source devel/setup.bash
exec bash
