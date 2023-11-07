[English](README.md) | __简体中文__

# 功能包 vision

## 1.介绍

本功能包为机器人视觉识别功能包，目前暂未完全实现。

## 2.结构

```
vision
├── onnx
    └── YOLOv6.onnx   # 轻量级 YOLOv6-Lite-S 模型
├── scripts
    └── gimbal.py     # 云台瞄准和射击
├── CMakeLists.txt
├── package.xml
├── README_cn.md
├── README.md
└── requirements.txt  # 依赖库列表
```

## 3.说明

本功能包暂未完全实现。

### 3.1 装甲板检测

通过对 [YOLOv6](https://github.com/meituan/yolv6) 进行轻量化以实现快速的装甲板目标检测。网络的参数量仅 0.55M，输入图像尺寸为 320 x 320，FLOPS 仅 0.56G，在配备一块 [Geforce RTX 2060 显卡](https://nvidia.cn/geforce/graphics-cards/rtx-2060)的机载电脑上的单图推理时间只需 4ms。

### 3.2 瞄准与击打

使用斜抛运动数学模型进行云台 yaw 角和 pitch 角的解算，从而实现机器人对敌方的瞄准与击打，详见[技术文档](../../images/vision/doc/shoot_cn.md)。

## 4.话题与服务

### 使用的服务

| 服务名称         | 节点名称 | 消息类型               | 说明           |
|:---------------:|:------:|:---------------------:|:--------------|
| /robot_id       | gimbal | decision/RobotID      | 查询我方队伍颜色 |
| /cmd_shoot      | gimbal | roborts_msgs/ShootCmd | 控制子弹发射    |
| /cmd_fric_wheel | gimbal | roborts_msgs/FricWhl  | 开启或关闭摩擦轮 |

### 发布的话题

| 话题名称           | 节点名称 | 消息类型                  | 说明                        |
|:-----------------:|:------:|:------------------------:|:---------------------------|
| /frame            | gimbal | sensor_msgs/Image        | 装甲板检测的结果图像，用于可视化 |
| /cmd_gimble_angle | gimbal | roborts_msgs/GimbalAngle | 控制机器人云台的角度           |
