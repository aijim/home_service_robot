#!/bin/sh

xterm -e " export ROBOT_INITIAL_POSE='-x 0 -y -3 -Y 0';  roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find pick_objects)/../worlds/my_room.world"&
sleep 5
xterm -e " export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find pick_objects)/../map/map.yaml;  roslaunch turtlebot_gazebo amcl_demo.launch "&
sleep 5
xterm -e "  roslaunch turtlebot_rviz_launchers view_navigation.launch "&
sleep 5
xterm -e "  roslaunch pick_objects pick_objects.launch"&
