[English](README.md) | __简体中文__

# 功能包 decision

## 1.介绍

本功能包为机器人决策功能包，目前暂未完全实现。

本功能包预期实现基于行为树的机器人决策方式。

## 2.结构

```
decision
├── src
    ├── gyroscope.cpp  # 摆尾策略节点
    └── robot_id.cpp   # 机器人 ID 服务节点
├── srv
    └── RobotID.srv    # 机器人 ID 服务类型
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.说明

本功能包暂未完全实现。

## 4.话题与服务

### 发布的话题

| 话题名称  | 节点名称    | 消息类型             | 说明              |
|:--------:|:---------:|:-------------------:|:-----------------|
| /cmd_vel | gyroscope | geometry_msgs/Twist | 控制机器人底盘的速度 |

### 提供的服务

| 服务名称    | 节点名称    | 消息类型          | 说明              |
|:----------:|:---------:|:----------------:|:-----------------|
| /gyroscope | gyroscope | std_srvs/SetBool | 开启或关闭摆尾状态  |
| /robot_id  | robot_id  | decision/RobotID | 查询机器人自身 ID  |
