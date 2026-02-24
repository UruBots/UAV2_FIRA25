# uav_mission_planner
This repository contains the code that will be used on WORLD FIRA for UAV competition.

### Install
In your worspace execute:
```
source devel/setup.bash
catkin_make
```

### Usage
* Launch: Execute launchers fo PX4, camera and the remaping of the topics.  
```
roslaunch uav_mission_planner launch.launch
```
* Mission Planner: Execute the mission planner menu.
```
roslaunch uav_mission_planner uav_mission_planner.launch
```