TPRM02
======

Group:
Jean-Luc Nacif Coelho - 2011049207
André Lloyd Harder - 2011048910

Makefile commands:

make: Copies relevant files to catkin workspace (must be in ~/catkin_ws) and runs catkin make.

make clean: Cleans up files from catkin workspace.

make run: Runs RRT algoritm with world 3. To change run settings modify run.sh. Please note that no specification for make run was provided.

How to run:

1- Run roscore in the backgound.
2- Run stageros to set up world: rosrun stage_ros stageros [world_location]
3- Run the explorer
	rosrun tp2 explore [origin x] [origin y] [max x] [max y] [Sensitivity]
	e.g., the following will work with worlds 0, 1, 3 and 4:
	rosrun tp2 explore 2 2 25 25 0.25
	
