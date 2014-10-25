all:
	cp -r . $(HOME)/catkin_ws/src/tp2
	cd $(HOME)/catkin_ws; \
	catkin_make

clean:
	cd $(HOME)/catkin_ws; \
	catkin_make clean
	rm -rf $(HOME)/catkin_ws/src/tp2
	echo "Cleanup complete!"

run:
	source /opt/ros/indigo/setup.bash
	source $(HOME)/catkin_ws/devel/setup.bash
	chmod 777 run.sh
	$(shell ./run.sh $<)
