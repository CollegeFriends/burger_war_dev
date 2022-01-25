#!/bin/bash
cd ${ROS_WORKSPACE}/src/burger_war_kit
gnome-terminal -- roscore
gnome-terminal -- bash scripts/sim_with_judge.sh