all:
	cp -r . $(HOME)/catkin_ws/src/tp1
	cp -r devel $(HOME)/catkin_ws/
	cd $(HOME)/catkin_ws; \
	catkin_make

clean:
	rm -rf $(HOME)/catkin_ws/src/tp1
	rm -rf $(HOME)/catkin_ws/devel/include/tp1
	echo "Cleanup complete!"

run:
	chmod 777 run.sh
	$(shell ./run.sh $<)
