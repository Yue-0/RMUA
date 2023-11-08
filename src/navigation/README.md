__English__ | [简体中文](README_cn.md)

# Package navigation

## 1.Introducion

This package is robot navigation package. The locating module uses the [AMCL algorithm](https://wiki.ros.org/amcl) that comes with ROS, and other modules are implemented independently.

## 2.Structure

```
navigation
├── launch
    ├── amcl.launch        # Locating algorithm
    ├── lidar.launch       # Start lidar
    ├── map.launch         # Map server
    ├── navigation.launch  # Start navigation
    └── planning.launch    # Planning algorithm
├── map
    ├── map.pgm            # RMUA 2021 map
    └── map.yaml           # Map yaml file
├── msg
    ├── Position.msg       # Robot self information
    └── Positions.msg      # Information about all robots
├── rviz
    └── rviz.rviz          # Configuration file for rviz
├── scripts
    └── sentry.py          # Sentry communicate node
├── src
    ├── lidar.cpp          # Lidar filter node
    ├── plan.cpp           # Path planning node
    ├── position.cpp       # Position publishing node
    └── vel.cpp            # velocity control node
├── srv
    └── plan.srv           # Navigation service
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.Illustrate

### 3.1 Lidar

Since the single-line lidar is installed in front of the robot's gimbal, the gimbal will block part of the lidar's scanning range, so the lidar data is filtered and only the lidar data 240 degrees in front of the robot is retained.

### 3.2 Locating

Use [AMCL algorithm](https://wiki.ros.org/amcl) to implement robot locating. All parameters of the algorithm are in [amcl.launch](launch/amcl.launch).

### 3.3 Path planning

In order to carry out fast path planning, the A* search algorithm is used for preliminary path planning. Since the path obtained by the A* algorithm is not optimal, key points in the preliminary path are selected and constructed into a directed weighted grahp. In directed weighted graph, the Dijkstra algorithm is used to optimize the path on the graph to obtain the final path.

![Path planning](../../images/navigation/planning.gif)

### 3.4 Velocity control

Since the robot chassis has the ability to move in all directions, the omnidirectional velocity can be calculated directly based on the results of path planning.

## 4.Topics and services

The following table only lists the topics and services of the nodes implemented by this project.

### Subscribed topics

| Topic             | Node   | Message               | Note                       |
|:-----------------:|:------:|:---------------------:|:---------------------------|
| /laser_scan       | lidar  | sensor_msgs/LaserScan | Raw data released by lidar |

### Services used

| Service   | Node     | Message          | Note                 |
|:---------:|:--------:|:----------------:|:---------------------|
| /robot_id | position | decision/RobotID | Query the robot's ID |

### Published topics

| Topic     | Node     | Message                | Note                                      |
|:---------:|:--------:|:----------------------:|:------------------------------------------|
| /scan     | lidar    | sensor_msgs/LaserScan  | Filtered lidar data                       |
| /path     | plan     | nav_msgs/Path          | Path planning result                      |
| /sentry   | sentry   | navigation/Positions   | Full field robots information from sentry |
| /cmd_vel  | vel      | geometry_msgs/Twist    | Control the velocity of the robot         |
| /costmap  | plan     | nav_msgs/OccupancyGrid | Cost map                                  |
| /position | position | navigation/Position    | Robot information from AMCL and RobotID   |

### Provided services

| Service | Node | Message         | Note                      |
|:-------:|:----:|:---------------:|:--------------------------|
| /plan   | plan | navigation/plan | Provid navigation service |
