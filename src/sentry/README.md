__English__ | [简体中文](README_cn.md)

# Package sentry

## 1.Introducion

This package is the sentry communication package, which defines the custom topics and services of this project.

## 2.Structure

```
sentry
├── msg
    ├── Points.msg     # Point cloud
    ├── Position.msg   # Robot self information
    └── Positions.msg  # Information about all robots
├── scripts
    └── client.py      # This script is not maintained by this repository
├── srv
    ├── plan.srv       # Navigation service
    └── RobotID.srv    # Robot ID service type
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.Custom topics

### 3.1 [Points.msg](msg/Points.msg)

This topic is used to record the results of fusion of front and rear radar data.

```
uint64 len    # Number of points
string frame  # Coordinate system name
float64[] x   # X coordinate
float64[] y   # Y coordinate
```

### 3.2 [Position.msg](msg/Position.msg)

This topic is used for the robot to send data to the sentry, including the robot's ID information and location information.

```
int8 id      // The ID of the robot, the values are -2, -1, 1, 2, positive numbers represent blue, negative numbers represent red
int32 x      // The x coordinate of the robot in the map coordinate system, unit: centimeters
int32 y      // The y coordinate of the robot in the map coordinate system, unit: centimeters
float64 yaw  // The rotation angle of the robot in the map coordinate system, unit: radians
```

### 3.3 [Positions.msg](msg/Positions.msg)

This topic is used for robot to receive data from sentry, including ID information and location information of all robots on the field.

```
uint8 len      // The number of all robots v the field
int8[] id      // IDs of all robots on the field
int32[] x      // The x-coordinates of all robots on the field in the map coordinate system, unit: centimeters
int32[] y      // The y-coordinates of all robots on the field in the map coordinate system, unit: centimeters
float64[] yaw  // The rotation angle of all robots on the field in the map coordinate system, unit: radians
```

## 4.Custom services

### 4.1 [plan.srv](srv/plan.srv)

This service is used to specify a navigation location or cancel navigation.

```
int32 x      // The x coordinate of the navigation target point in the map coordinate system, unit: centimeters
int32 y      // The y coordinate of the navigation target point in the map coordinate system, unit: centimeters
float64 yaw  // The rotation angle of the navigation target point in the map coordinate system, unit: radians
---
bool result  // True if the navigation point is set successfully or canceled successfully. Both x and y are 0 to cancel the navigation.
```

### 4.2 [RobotID.srv](srv/RobotID.srv)

This service is used to request color and number information for a robot.

```
bool num    // Whether to request number information
bool color  // Whether to request color information
---
int8 id     // Robot ID, number 1 if no numbering information is requested, blue if no color information is requested
```
