#!/bin/bash

# Reset paramators
rosnode kill send_id_to_judge
rosnode kill /enemy_bot/send_id_to_judge
rosnode kill /enemy_bot/turtlebot3_teleop_keyboard
rosnode cleanup
rosservice call /gazebo/reset_simulation "{}"

# Add permission
cd ~/catkin_ws/src/burger_war_dev/burger_war_dev
chmod +x scripts/*.py

# reset judge server
cd ~/catkin_ws/src/burger_war_kit
bash judge/test_scripts/reset_server.sh judge/marker_set/sim.csv localhost:5000 you enemy