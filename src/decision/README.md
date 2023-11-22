__English__ | [简体中文](README_cn.md)

# Package decision

## 1.Introducion

This package is robot decision package, only 1v1 functionality is implemented currently.

## 2.Structure

```
decision
├── launch
    ├── 1v1.launch       # Start 1v1 game
    └── decision.launch  # Run decision nodes
├── src
    ├── ai.cpp           # Decision node
    ├── aim.cpp          # Enemy search node
    ├── gyroscope.cpp    # Gyroscope node
    └── robot_id.cpp     # Robot ID service node
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.Illustrate

Not updated yet.

## 4.Topics and services

### Subscribed topics

| Topic     | Node | Message                | Note                                     |
| /sentry   | aim  | sentry/Positions       | Full field robot information from sentry |
| /costmap  | ai   | nav_msgs/OccupancyGrid | Cost map                                 |
| /position | aim  | sentry/Position        | Robot's own position information         |

### Services used

| Service | Node | Message     | Note                                    |
|:-------:|:----:|:-----------:|:----------------------------------------|
| /plan   | ai   | sentry/plan | Specify the target point for navigation |

### Published topics

| Topic        | Node      | Message                                 | Note                                                         |
|:------------:|:---------:|:---------------------------------------:|:-----------------------------------------------------------|
| /enemy       | aim       | sentry/Position                         | The location information of the currently locked enemy robot |
| /cmd_vel     | gyroscope | geometry_msgs/Twist                     | Control the speed of the robot chassis                       |
| /initialpose | ai        | geometry_msgs/PoseWithCovarianceStamped | Robot initial pose                                           |

### Provided services

| Service    | Node      | Message          | Note                     |
|:----------:|:---------:|:----------------:|:-------------------------|
| /start     | ai        | std_srvs/SetBool | Start or pause decision  |
| /robot_id  | robot_id  | sentry/RobotID   | Query the robot's own ID |
| /gyroscope | gyroscope | std_srvs/SetBool | Turn or off gyroscope    |
