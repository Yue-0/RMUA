__English__ | [简体中文](README_cn.md)

# Package decision

## 1.Introducion

This package is robot visual recognition package that provides armor plate detection and shooting functions.

## 2.Structure

```
vision
├── onnx
    └── YOLOv6.onnx   # Lightweight YOLOv6-Lite-S model
├── scripts
    └── gimbal.py     # Gimbal aiming and shooting
├── CMakeLists.txt
├── package.xml
├── README_cn.md
├── README.md
└── requirements.txt  # Requirements list
```

## 3.Illustrate

### 3.1 Armor plate detection

By lightweigting [YOLOv6](https://github.com/meituan/yolv6) to achieve fast armor plate detection. The number of parameters of the lightweight model is only 0.55M. When the input image size is 320 x 320, the FLOPS is only 0.56G. The single-image inference time on a computer equipped with a [Geforce RTX 2060](https://nvidia.cn/geforce/graphics-cards/rtx-2060) only takes 4ms.

![Armor plate detection](../../images/vision/detection.gif)

### 3.2 Aim and shoot

Use the mathematical model of oblique throwing motion to calculate the rotation angle of the gimbal, so as to realize the robot's aiming and shooting of the enemy. See [technical documentation](../../images/vision/doc/shoot.md) for details.

## 4.Topics and services

### Services used

| Service         | Node   | Message               | Note                               |
|:---------------:|:------:|:---------------------:|:-----------------------------------|
| /robot_id       | gimbal | sentry/RobotID        | Check the color of our team.       |
| /cmd_shoot      | gimbal | roborts_msgs/ShootCmd | Control bullet shoot.              |
| /cmd_fric_wheel | gimbal | roborts_msgs/FricWhl  | Turn on or off the friction wheel. |

### Published topics

| Topic             | Node   | Message                  | Note                                                   |
|:-----------------:|:------:|:------------------------:|:-------------------------------------------------------|
| /frame            | gimbal | sensor_msgs/Image        | Result image of armor plate detection for visualization |
| /cmd_gimble_angle | gimbal | roborts_msgs/GimbalAngle | Control the angle of the robot's gimbal            |
