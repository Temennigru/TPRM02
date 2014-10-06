all:
	cp -r . $(HOME)/catkin_ws/src/tp1
	cp -r devel $(HOME)/catkin_ws/
	cd $(HOME)"/catkin_ws"; \
	catkin_make
