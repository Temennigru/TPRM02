TPRM01
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
3- Run controller: 
         Format:
          	(1) rosrun tp1 control [origin x] [origin y] [Sensitivity] "omni" [KP] [KD] [KI]
          	(2) rosrun tp1 control [origin x] [origin y] [Sensitivity] "diff" [KR] [KA] [KL] [KW]
        Note: Suggested 1 0 0 for KP, KD and KI respectively.
        Note: Suggested 1 9 1 1 for KR, KA, KL and KW respectively
4- Run path finder:
         Format:
	          (1) rosrun tp1 path [srcX] [srcY] [dstX] [dstY] [pgm file] [padding] RRT 
	          (2) rosrun tp1 path [srcX] [srcY] [dstX] [dstY] [pgm file] [padding] VisibilityGraph
        Note: Suggested 5 for padding
  
make run runs run.sh which has the following format:



roscore &
sleep 2; rosrun stage_ros stageros $HOME/catkin_ws/src/tp1/world/omniworld_3.world &
sleep 5; rosrun tp1 control 2 2 0.7 omni 1 0 0 &
sleep 5; rosrun tp1 path 20 20 220 20 src/tp1/world/tp1_floor3.pgm 7 RRT


which is equivalent to:


roscore &
sleep 2; rosrun stage_ros stageros $HOME/catkin_ws/src/tp1/world/omniworld_3.world &
sleep 5; rosrun tp1 control [origin x] [origin y] [Sensitivity] [controller_type] [KP] [KD] [KI] &
sleep 5; rosrun tp1 path [srcX] [srcY] [dstX] [dstY] [pgm file] [padding] [pathing_type]


if you would like to run the path finder through make run, just change the parameters in the file.
