all:
	cp -r . $(HOME)/catkin_ws/src/robot
	cd $(HOME)"/catkin_ws"; \
	mkdir devel/include; \
	mkdir devel/include/tp1; \
	cp src/robot/cchord.h devel/include/tp1
	catkin_make
