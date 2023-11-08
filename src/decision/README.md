__English__ | [简体中文](README_cn.md)

# Package decision

## 1.Introducion

This package is robot decision package, which is not fully implemented at this time.

This package is expected to implement a behavior tree based robot decision making approach.

## 2.Structure

```
decision
├── scripts
    └── sp.py          # Sentry communicate node
├── src
    ├── gyroscope.cpp  # Gyroscope node
    └── robot_id.cpp   # Robot ID service node
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.Illustrate

This package is not fully implemented yet.

## 4.Topics and services

### Published topics

| Topic    | Node      | Message             | Note                               |
|:--------:|:---------:|:-------------------:|:-----------------------------------|
| /cmd_vel | gyroscope | geometry_msgs/Twist | Control the velocity of the robot. |

### Provided services

| Service    | Node      | Message          | Note                          |
|:----------:|:---------:|:----------------:|:------------------------------|
| /gyroscope | gyroscope | std_srvs/SetBool | Turn on or off the gyroscope. |
| /robot_id  | robot_id  | sentry/RobotID   | Query the robot's ID.         |
