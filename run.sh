echo "Setting up ROS"
cd $HOME/catkin_ws/; \
echo "Running roscore"; roscore &
sleep 2; echo "Running stageros"; roslaunch stage_ros stageros src/tp2/world/omniworld_0.world &
sleep 4; echo "Running controler"; rosrun tp2 explore 2 2 25 25 0.25 1 1
pkill roscore
