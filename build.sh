#!/bin/bash
cd src/swarm-primitive-dp/swarm_simulator
catkin_make
source devel/setup.bash
cd ../../..
catkin_make
source devel/setup.bash
exec bash
