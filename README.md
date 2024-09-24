# Art Explorer

The Art explorer is a ROS2 humble based node generated enable a Turtlebot3 to identify Apriltags and Images by exploring a 5x5 maze generated in gazebo sim. 

## Installation

By successfully cloning and building this workspace and having the following packages installed the robot should be ready to explore the maze.

* Gazebo Sim
* SLAM Toolbox
* Nav2 
* apriltag_ros package
* Yolov8 

## Usage

First the maze is to be generated from the art gallery workspace in gazebo: 
```bash
ros2 launch tb3_gazebo arams.launch.py
```
then the nav2 server from my_robot_navigation and the localization with the SLAM toolbox from my_robot_slam package is initialized:
```bash
ros2 launch my_robot_navigation robot_nav.launch.py
ros2 launch my_robot_slam localization.launch.yaml
```
from launching the nav2 servers Rviz should also open and by launch localization the map should load. The initial position estimate of the is to be given in Rviz.
Now the files for Apriltag and Image detection are to be run and they are as follows:
```bash
ros2 launch apriltag_ros apriltag.launch.yml
ros2 run yolo_ros yolo_to_ros
```
After the initalisation of the setup now the auto explorer node is run to let the robot explore the arena: 
```bash
ros2 run my_robot_navigation art_explorer
```
With running these launch files and nodes the robot shall start exploring the maze and identifying the Apriltags and images.

## Team

Seyedehmona Basiri – 3197979 - seyedehmona.basiri@alumni.fh-aachen.de

Amitojsingh Arora - 3238617 - amitojsingh.arora@alumni.fh-aachen.de
