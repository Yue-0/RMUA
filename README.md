<div align="center">
    <img src="images/robot.png" width="431" height="323" />
</div>

__English__ | [简体中文](README_cn.md)

# Multi-agent 2v2 automatic confrontation

## 1.Introducion

This project is one of the important topics in major projects, aiming to realize multi-agent 2v2 automatic confrontation.
This project uses a total of 4 omnidirectional mobile robots to conduct fully automatic confrontation on the [competition map of RMUA2021](src/navigation/map/map.pgm).
All codes in this project run on Ubuntu20.04 and rely on [ROS-noetic](http://wiki.ros.org/noetic).

## 2.Structure

| Package name                 | Function                   |
|:----------------------------:|:--------------------------:|
| [decision](src/decision)     | Robt decision              |
| [navigation](src/navigation) | Navigation and positioning |
| [sentry](src/sentry)         | Define sentry messages     |
| [vision](src/vision)         | Visual identity            |

Custom topics and services are defined in the sentry package, and other packages need to rely on this package. For the specific structure, functions and principles of each package, please refer to the README file inside the package.

```
src
├── decision
├── navigation
├── sentry
└── vision
```

## 3.Requirements

* The robot used in this project is the [RoboMaster2020 standard AI robot](https://www.robomaster.com/zh-CN/products/components/detail/2499). You need to download and compile [RoboRTS](https://github.com/RoboMaster/RoboRTS-Base) as the underlying driver.
* This project uses an [Intel RealSense](https://www.intelrealsense.com/) depth camera as a visual sensor, installed under the gimbal barrel, and uses Python to read image data, which requires the pyrealsense2 library.
* This project uses two single-line lidars, which are installed in front and behind the robot. The ROS driver corresponding to the lidar needs to be installed.
* Some codes of some packages in this project are written in Python >= 3.8, and the dependencies are given in requirements.txt in the package directory.

## 4.Build

Before compiling this project, please make sure you have successfully compiled the ROS driver for RoboRTS and radar.
Clone and compile this project using the following code:

```shell
git clone https://github.com/Yue-0/RMUA.git
cd ./RMUA/src
catkin_init_workspace
cd vision
pip install -r requirements.txt
cd ../..
catkin_make --only-pkg-with-deps sentry
catkin_make -DCATKIN_WHITELIST_PACKAGES="decision;navigation;vision"
source devel/setup.bash
```

## 5.Run 1v1

After successful compilation, move the robot to the starting point and use the following commands to make the robot ready for the 1v1 competition.

```shell
roslaunch decision 1v1.launch
```

Running the above command will start rviz. After the robot positioning is completed, enter the following command in another terminal to start decision-making.

```shell
rosservice call /start 1
```
