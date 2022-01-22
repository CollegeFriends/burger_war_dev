#!/bin/bash

# Reset
cd ~/catkin_ws/src/burger_war_dev
bash burger_war_dev/bat/reset.sh

# Start
roslaucnh burger_war_dev restart.laucnh
bash burger_war_dev/bat/start.sh