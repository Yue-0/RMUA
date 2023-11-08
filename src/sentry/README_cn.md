[English](README.md) | __简体中文__

# 功能包 sentry

## 1.介绍

This package is the sentry communication package, which defines the custom topics and services of this project.

## 2.结构

```
sentry
├── msg
    ├── Position.msg   # 机器人自身信息
    └── Positions.msg  # 所有机器人的信息
├── srv
    ├── plan.srv       # 导航服务
    └── RobotID.srv    # 机器人 ID 服务
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.自定义话题

### 3.1 [Position.msg](msg/Position.msg)

本话题用于机器人向哨岗发送数据，包含机器人自身的 ID 信息和位置信息。

```
int8 id      // 机器人的 ID，取值为 -2、-1、1、2，正数表示蓝色，负数表示红色
int32 x      // 机器人在 map 坐标系下的 x 坐标，单位：厘米
int32 y      // 机器人在 map 坐标系下的 y 坐标，单位：厘米
float64 yaw  // 机器人在 map 坐标系下的旋转角度，单位：弧度
```

### 3.2 [Positions.msg](msg/Positions.msg)

本话题用于机器人接收来自哨岗的数据，包含全场所有机器人的 ID 信息和位置信息。

```
uint8 len      // 机器人个数
int8[] id      // 所有机器人的 ID，取值为 -2、-1、1、2，正数表示蓝色，负数表示红色
int32[] x      // 所有机器人在 map 坐标系下的 x 坐标，单位：厘米
int32[] y      // 所有机器人在 map 坐标系下的 y 坐标，单位：厘米
float64[] yaw  // 所有机器人在 map 坐标系下的旋转角度，单位：弧度
```

## 4.自定义服务

### 4.1 [plan.srv](srv/plan.srv)

本服务用于指定导航位置或取消导航。

```
int32 x      // 导航目标点在 map 坐标系下的 x 坐标，单位：厘米
int32 y      // 导航目标点在 map 坐标系下的 y 坐标，单位：厘米
float64 yaw  // 导航目标点在 map 坐标系下的旋转角度，单位：弧度
---
bool result  // 导航点设置成功或取消成功则为真，x 和 y 都为 0 表示取消导航
```

### 4.2 [RobotID.srv](srv/RobotID.srv)

本服务用于请求机器人的颜色和编号信息。

```
bool num    // 是否请求编号信息
bool color  // 是否请求颜色信息
---
int8 id     // 机器人 ID，如果不请求编号信息，则编号为 1，如果不请求颜色信息，则为蓝色
```
