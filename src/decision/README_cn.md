[English](README.md) | __简体中文__

# 功能包 decision

## 1.介绍

本功能包为机器人决策功能包，目前暂未完全实现。

本功能包预期实现基于行为树的机器人决策方式。

## 2.结构

```
decision
├── scripts
    └── sp.py          # 哨岗通信节点
├── src
    ├── gyroscope.cpp  # 摆尾策略节点
    └── robot_id.cpp   # 机器人 ID 服务节点
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.说明

本功能包暂未完全实现。

## 4.话题与服务

### 订阅的话题

| 话题名称   | 节点名称 | 消息类型         | 说明              |
|:---------:|:--------:|:---------------:|:-----------------|
| /position | sentry   | sentry/Position | 机器人自身位置信息 |

### 发布的话题

| 话题名称  | 节点名称  | 消息类型             | 说明                   |
|:--------:|:---------:|:-------------------:|:-----------------------|
| /sentry  | sentry    | sentry/Positions    | 来自哨岗的全场机器人信息 |
| /cmd_vel | gyroscope | geometry_msgs/Twist | 控制机器人底盘的速度    |

### 提供的服务

| 服务名称    | 节点名称    | 消息类型          | 说明              |
|:----------:|:---------:|:----------------:|:-----------------|
| /gyroscope | gyroscope | std_srvs/SetBool | 开启或关闭摆尾状态  |
| /robot_id  | robot_id  | sentry/RobotID   | 查询机器人自身 ID  |
