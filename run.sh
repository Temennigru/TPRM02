echo "Setting up ROS"
cd $HOME/catkin_ws/; \
echo "Running roscore"; roscore &
sleep 2; echo "Running stageros"; rosrun stage_ros stageros $HOME/catkin_ws/src/tp1/world/omniworld_3.world &
sleep 5; echo "Running controler"; rosrun tp1 control 2 2 0.7 omni 1 0 0 &
sleep 5; echo "Running pathfinder"; rosrun tp1 path 20 20 220 20 src/tp1/world/tp1_floor3.pgm 7 RRT
