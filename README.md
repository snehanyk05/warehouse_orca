# warehouse
Warehouse ORCA Planning

#1 
On terminal 1 - roscore

#2
On terminal 2 - rosrun warehouse_manager warehouse_master

#3
On terminal 3 - roslaunch warehouse_orca turtlebot_stage_rrt.launch (In case of RRT)
or
On terminal 3 - roslaunch warehouse_orca turtlebot_stage_astar.launch (In case of RRT)
or in case of Dynamic astar dont run this step (Just follow dynamic_gloabl_planner and death_star package instructions)
#4
On terminal 4 - cd into warehouse_orca/src and run
python multi_process_agents.py

There are 4 files in the src folder other than multi_process_agents.py
1. turtle_instance.py (without laser)
2. turtle_instance_laser.py 
3. turtle_instance_astar.py (global planning)
4. turtle_instance_laser_astar.py( with astar and lidar)
5. turtle_instance_laser_rrt.py( with rrt and lidar)
6. turtle_instance_laser_death_star.py( with dynamic astar and lidar)
You can switch these file in multi_process_agents.py, by default it is turtle_instance_laser.py 
