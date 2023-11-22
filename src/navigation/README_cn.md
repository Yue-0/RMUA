[English](README.md) | __简体中文__

# 功能包 navigation

## 1.介绍

本功能包为机器人导航功能包，其中定位模块使用了 ROS 自带的 [AMCL 算法](https://wiki.ros.org/amcl)，其他模块均自主实现。

## 2.结构

```
navigation
├── launch
    ├── amcl.launch        # 定位算法
    ├── lidar.launch       # 启动雷达
    ├── map.launch         # 地图服务
    ├── navigation.launch  # 启动导航
    └── planning.launch    # 规划算法
├── map
    ├── map.pgm            # RMUA 2021 地图
    └── map.yaml           # 地图描述文件
├── rviz
    └── rviz.rviz          # rviz 配置文件
├── scripts
    └── mouse.py           # 鼠标控制节点，即将弃用
├── src
    ├── as.hpp             # A* 算法
    ├── lidar.cpp          # 雷达滤波节点
    ├── plan.cpp           # 路径规划节点
    ├── plan.hpp           # 以 A* 为基础的 Dijkstra 算法
    ├── points.cpp         # 点云构建节点
    ├── position.cpp       # 位置发布节点
    └── vel.cpp            # 速度控制节点
├── CMakeLists.txt
├── package.xml
├── README_cn.md
└── README.md
```

## 3.说明

### 3.1 雷达

使用两个单线激光雷达，分别安装在机器人的正前方和正后方，实现全向感知。

### 3.2 定位

使用 [AMCL 算法](https://wiki.ros.org/amcl)实现机器人定位，算法的所有参数都在 [amcl.launch](launch/amcl.launch) 中。

### 3.3 路径规划

为了进行快速的路径规划，使用 A* 启发式搜索算法进行初步路径规划，由于 A* 算法得到的路径并不是最优的，因此挑选出初步路径中的关键点，并将这些关键点构建成一张有向带权图，在图上使用 Dijkstra 算法优化路径，得到最终的路径。

![路径规划](../../images/navigation/planning.gif)

### 3.4 速度控制

由于机器人底盘具有全向移动的能力，所以直接根据路径规划的结果计算全向速度即可。

## 4.话题和服务

下表只列出了本项目实现的节点的话题与服务，不包括 ctrl 节点（因为该节点即将弃用）。

### 订阅的话题

| 话题名称 | 节点名称 | 消息类型         | 说明                       |
|:-------:|:-------:|:----------------:|:--------------------------|
| /enemy  | vel     | sentry/Position  | 来自决策节点的敌人位置信息   |
| /sentry | plan    | sentry/Positions | 来自哨岗的全场机器人位置信息 |

### 使用的服务

| 服务名称   | 节点名称  | 消息类型       | 说明             |
|:---------:|:--------:|:--------------:|:-----------------|
| /robot_id | position | sentry/RobotID | 查询机器人自身 ID |

### 发布的话题

| 话题名称   | 节点名称  | 消息类型               | 说明                           |
|:---------:|:--------:|:----------------------:|:------------------------------|
| /scan     | lidar    | sentry/Points          | 融合后的点云数据                |
| /path     | plan     | nav_msgs/Path          | 路径规划结果                   |
| /scan1    | lidar    | sensor_msgs/LaserScan  | 过滤后的前置雷达数据            |
| /scan2    | lidar    | sensor_msgs/LaserScan  | 过滤后的后置雷达数据            |
| /cmd_vel  | vel      | geometry_msgs/Twist    | 控制机器人底盘的速度            |
| /costmap  | plan     | nav_msgs/OccupancyGrid | 代价地图                       |
| /position | position | sentry/Position        | 来自 AMCL 和 RobotID 的自身信息 |

### 提供的服务

| 服务名称 | 节点名称 | 消息类型     | 说明        |
|:-------:|:------:|:-----------:|:-----------|
| /plan   | plan   | sentry/plan | 提供导航服务 |
