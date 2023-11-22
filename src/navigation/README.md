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
├── rviz
    └── rviz.rviz          # Configuration file for rviz
├── scripts
    └── mouse.py           # Mouse control node, will be deprecated soon
├── src
    ├── as.hpp             # A* algorithm
    ├── lidar.cpp          # Lidar filter node
    ├── plan.cpp           # Path planning node
    ├── plan.hpp           # A* based Dijkstra algorithm
    ├── points.cpp         # Point cloud node
    ├── position.cpp       # Position publishing node
    └── vel.cpp            # Velocity control node
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.Illustrate

### 3.1 Lidar

We use two single-line lidars, which are installed directly in front and behind the robot to achieve omnidirectional sensing.

### 3.2 Locating

Use [AMCL algorithm](https://wiki.ros.org/amcl) to implement robot locating. All parameters of the algorithm are in [amcl.launch](launch/amcl.launch).

### 3.3 Path planning

In order to carry out fast path planning, the A* search algorithm is used for preliminary path planning. Since the path obtained by the A* algorithm is not optimal, key points in the preliminary path are selected and constructed into a directed weighted grahp. In directed weighted graph, the Dijkstra algorithm is used to optimize the path on the graph to obtain the final path.

![Path planning](../../images/navigation/planning.gif)

### 3.4 Velocity control

Since the robot chassis has the ability to move in all directions, the omnidirectional velocity can be calculated directly based on the results of path planning.

## 4.Topics and services

The following table only lists the topics and services of the nodes implemented by this project, excluding the ctrl node ,because this node will be deprecated.

### Subscribed topics

| Topic   | Node | Message          | Note                                                 |
|:-------:|:----:|:----------------:|:-----------------------------------------------------|
| /enemy  | vel  | sentry/Position  | Enemy location information from decision nodes       |
| /sentry | plan | sentry/Positions | Full-site robot position information from the sentry |

### Services used

| Service   | Node     | Message        | Note                 |
|:---------:|:--------:|:--------------:|:---------------------|
| /robot_id | position | sentry/RobotID | Query the robot's ID |

### Published topics

| Topic     | Node     | Message                | Note                                      |
|:---------:|:--------:|:----------------------:|:------------------------------------------|
| /scan     | lidar    | sentry/Points          | Fusion point cloud data                   |
| /path     | plan     | nav_msgs/Path          | Path planning result                      |
| /scan2    | lidar    | sensor_msgs/LaserScan  | Filtered rear lidar data                  |
| /scan1    | lidar    | sensor_msgs/LaserScan  | Filtered front lidar data                 |
| /cmd_vel  | vel      | geometry_msgs/Twist    | Control the velocity of the robot         |
| /costmap  | plan     | nav_msgs/OccupancyGrid | Cost map                                  |
| /position | position | sentry/Position        | Robot information from AMCL and RobotID   |

### Provided services

| Service | Node | Message     | Note                      |
|:-------:|:----:|:-----------:|:--------------------------|
| /plan   | plan | sentry/plan | Provid navigation service |
