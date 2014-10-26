echo "Setting up ROS"
sudo $HOME/catkin_ws/devel/setup.bash
cd $HOME/catkin_ws/; \
pkill roscore; \
echo "Running roscore"; roscore & \
sleep 2; echo "Running stageros"; rosrun stage_ros stageros src/tp2/world/omniworld_3.world & \
sleep 4; echo "Running controler"; rosrun tp2 explore 13 2 25 25 0.25 1 1 \
pkill roscore
