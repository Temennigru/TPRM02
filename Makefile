all:
	cp -r . $(HOME)"/catkin_ws/src/robot/Graph"
	cd $(HOME)"/catkin_ws"
	catkin_make