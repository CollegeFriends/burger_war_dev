#!/bin/bash
cd ~/catkin_ws/src/burger_war_dev/burger_war_dev/scripts
chmod +x *.py
cd ~/catkin_ws/src/burger_war_kit/
bash judge/test_scripts/set_running.sh localhost:5000
roslaunch burger_war sim_robot_run.launch enemy_level:=$VALUE_L