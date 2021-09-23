# Project 5: Home Service Robot 
In this project, a home service robot is simulated to pick up a object at pick-up zone and then drop off the object when it reachs drop-off zone. 

## Prerequisites
1. ROS (Kinetic), Gazebo on Linux
2. CMake & g++/gcc, C++11
3. Install xterm
```
sudo apt-get install xterm
```
4. Install some dependencies
```
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/slam_gmapping
git clone https://github.com/turtlebot/turtlebot
git clone https://github.com/turtlebot/turtlebot_interactions
git clone https://github.com/turtlebot/turtlebot_simulator
cd ~/catkin_ws/
source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo
catkin_make
source devel/setup.bash
```

## Build and Launch
1. Clone the project and build it
```
cd ~/catkin_ws/src
git clone https://github.com/aijim/home_service_robot.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
2. Make shell scripts executable
```
cd ~/catkin_ws/src/home_service_robot/scripts
chmod a+x *
```
3. Execute scripts
```
./home_service.sh
```
