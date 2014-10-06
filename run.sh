cd $HOME/catkin_ws/; \
roscore &
sleep 2; rosrun stage_ros stageros $HOME/catkin_ws/src/tp1/world/omniworld_3.world &
sleep 5; rosrun tp1 control 2 2 3 omni 1 0 0 &
sleep 5; rosrun tp1 path 20 20 220 20 src/tp1/world/tp1_floor3.pgm 5 RRT
