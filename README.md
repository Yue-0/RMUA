<div align="center">
    <img src="images/robot.png" width="431" height="323" />
</div>

__English__ | [简体中文](README_cn.md)

# Multi-agent 2v2 automatic confrontation

## 1.Introducion

This project is one of the important topics in major projects, aiming to realize multi-agent 2v2 automatic confrontation.
This project uses a total of 4 omnidirectional mobile robots to conduct fully automatic confrontation on the competition map of [RMUA2021](https://icra2021.org/competitions/dji-robomaster-ai-challenge).
All codes in this project run on Ubuntu20.04 and rely on [ROS-noetic](http://wiki.ros.org/noetic).

## 2.Structure

| Package name             | Function                   |
|:------------------------:|:--------------------------:|
| [decision](decision)     | Robt decision              |
| [navigation](navigation) | Navigation and positioning |
| [vision](vision)         | Visual identity            |

For the specific structure, functions and principles of any packages, please refer to the README file inside the package.

```
src
├── decision
├── navigation
└── vision
```

## 3.Requirements

* The robot used in this project is the [RoboMaster2020 standard AI robot](https://www.robomaster.com/zh-CN/products/components/detail/2499). You need to download and compile [RoboRTS](https://github.com/RoboMaster/RoboRTS) as the underlying driver.
* This project uses an [Intel RealSense](https://www.intelrealsense.com/) depth camera as a visual sensor, installed under the gimbal barrel, and uses Python to read image data, which requires the pyrealsense2 library.
* This project uses a single-line lidar, which is installed upside down about 20 cm directly in front of the center of the robot. It is necessary to install the ROS driver corresponding to the lidar.
* Some codes of some packages in this project are written in Python >= 3.8, and the dependencies are given in requirements.txt in the package directory.

## 4.Build

Before compiling this project, please make sure you have successfully compiled the ROS driver for RoboRTS and radar.
Clone and compile this project using the following code:

```shell
git clone https://github.com/Yue-0/RMUA.git
cd ./RMUA/src
catkin_init_workspace
cd ..
catkin_make
```
