#!/usr/bin/env bash

for i in $(11311 11312 11313 11314); do
  xterm -e source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://localhost:$i && rosland etu_simulation time_moves.launch
