#!/bin/bash
cd ${ROS_WORKSPACE}/src/burger_war_kit
rosparam set /use_sim_time true
gnome-terminal -- bash scripts/sim_with_judge.sh