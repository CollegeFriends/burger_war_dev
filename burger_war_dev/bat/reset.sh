 
#!/bin/bash
cd ~/catkin_ws/src/burger_war_kit
rosservice call /gazebo/reset_simulation "{}"
bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy