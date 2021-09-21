#!/bin/sh

xterm -e " export ROBOT_INITIAL_POSE='-x 0 -y -3 -Y 0'; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/home_service_robot/worlds/my_room.world"&
sleep 5
xterm -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch "&
sleep 5
xterm -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch "&
sleep 5
xterm -e " source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch "