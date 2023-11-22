[English](README.md) | __简体中文__

# 功能包 decision

## 1.介绍

本功能包为机器人决策功能包，目前仅实现了 1v1 功能。

## 2.结构

```
decision
├── launch
    ├── 1v1.launch       # 开始 1v1 比赛
    └── decision.launch  # 开启决策节点
├── src
    ├── ai.cpp           # 决策节点
    ├── aim.cpp          # 敌军搜索节点
    ├── gyroscope.cpp    # 摆尾策略节点
    └── robot_id.cpp     # 机器人 ID 服务节点
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.说明

暂未更新。

## 4.话题与服务

### 订阅的话题

| 话题名称   | 节点名称 | 消息类型               | 说明                   |
|:---------:|:-------:|:----------------------:|:-----------------------|
| /sentry   | aim     | sentry/Positions       | 来自哨岗的全场机器人信息 |
| /costmap  | ai      | nav_msgs/OccupancyGrid | 代价地图               |
| /position | aim     | sentry/Position        | 机器人自身位置信息      |

### 使用的服务

| 服务名称 | 节点名称 | 消息类型    | 说明            |
|:-------:|:-------:|:-----------:|:-------------- |
| /plan   | ai      | sentry/plan | 指定导航的目标点 |

### 发布的话题

| 话题名称      | 节点名称   | 消息类型                                | 说明                         |
|:------------:|:---------:|:---------------------------------------:|:----------------------------|
| /enemy       | aim       | sentry/Position                         | 当前锁定的敌方机器人的位置信息 |
| /cmd_vel     | gyroscope | geometry_msgs/Twist                     | 控制机器人底盘的速度          |
| /initialpose | ai        | geometry_msgs/PoseWithCovarianceStamped | 机器人初始位姿               |

### 提供的服务

| 服务名称    | 节点名称   | 消息类型         | 说明              |
|:----------:|:---------:|:----------------:|:------------------|
| /start     | ai        | std_srvs/SetBool | 开启或暂停决策     |
| /robot_id  | robot_id  | sentry/RobotID   | 查询机器人自身 ID  |
| /gyroscope | gyroscope | std_srvs/SetBool | 开启或关闭摆尾状态 |
