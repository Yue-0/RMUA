<div align="center">
    <img src="images/robot.png" width="431" height="323" />
</div>

[English](README.md) | __简体中文__

# 多智能体2v2自动对抗

## 1.介绍

本工程是重大项目中的重要课题之一，旨在实现多智能体 2v2 自动对抗。
本工程共使用 4 台全向移动机器人在 [RMUA 2021 的比赛地图](src/navigation/map/map.pgm)上进行全自动对抗。
本工程所有代码在 Ubuntu20.04 运行，依赖 [ROS-noetic](http://wiki.ros.org/noetic)。

## 2.结构

| 功能包名称                    | 功能        |
|:----------------------------:|:----------:|
| [decision](src/decision)     | 机器人决策  |
| [navigation](src/navigation) | 导航和定位  |
| [sentry](src/sentry)         | 定义哨岗消息 |
| [vision](src/vision)         | 视觉识别    |

sentry 功能包中定义了自定义话题和服务，其他功能包都需要依赖此功能包。各个功能包的具体结构、功能和原理请参考功能包内部的 README 文件。

```
src
├── decision
├── navigation
├── sentry
└── vision
```

## 3.依赖

* 本工程使用的机器人为 [RoboMaster 2020 标准版 AI 机器人](https://www.robomaster.com/zh-CN/products/components/detail/2499)，需要先[下载并编译 RoboRTS](https://github.com/RoboMaster/RoboRTS) 作为底层驱动。
* 本工程使用一个 [Intel RealSense](https://www.intelrealsense.com/) 深度相机作为视觉传感器，安装在云台炮管的下方，并使用 Python 读取图像数据，需要依赖 pyrealsense2 库。
* 本工程使用两个单线激光雷达，分别安装在机器人的前方和后方，需要安装对应雷达的 ROS 驱动。
* 本工程一些功能包的部分代码使用 Python >= 3.8 编写，依赖在功能包目录下的 requirements.txt 中给出。

## 4.编译

编译本工程前，请先确保已经成功编译 RoboRTS 和雷达的 ROS 驱动。
使用以下代码克隆并编译本工程：

```shell
git clone https://github.com/Yue-0/RMUA.git
cd ./RMUA/src
catkin_init_workspace
cd ./vision
pip install -r requirements.txt
cd ../..
catkin_make --only-pkg-with-deps sentry
catkin_make -DCATKIN_WHITELIST_PACKAGES="decision;navigation;vision"
```

## 5.运行

编译成功后，将机器人移动至起始点，使用以下指令使机器人进入 1v1 比赛的准备状态

```shell
roslaunch decision 1v1.launch
```

运行以上指令会启动 rviz，等待机器人定位完成后，在另一个终端中输入以下指令开始决策

```shell
rosservice call /start 1
```
