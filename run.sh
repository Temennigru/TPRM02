echo "Setting up ROS"
cd $HOME/catkin_ws/; \
echo "Running roscore"; roscore & \
sleep 2; echo "Running stageros"; rosrun stage_ros stageros src/tp2/world/omniworld_3.world & \
sleep 4; echo "Running controler"; rosrun tp2 explore 2 2 25 25 0.25
pkill roscore
pkill stage_ros
