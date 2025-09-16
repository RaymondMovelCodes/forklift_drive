# forklift_drive ROS Package

[![Version](https://img.shields.io/badge/version-1.0.1-blue.svg)](https://github.com/raymond-robotics/forklift_drive)
[![ROS](https://img.shields.io/badge/ROS-Melodic%20%7C%20Noetic-green.svg)](http://wiki.ros.org/)
[![License](https://img.shields.io/badge/license-Proprietary-red.svg)](LICENSE)

---

## 📋 目录 | Table of Contents

- [中文说明](#中文说明)
  - [功能描述](#功能描述)
  - [系统要求](#系统要求)
  - [安装与编译](#安装与编译)
  - [ROS节点信息](#ros节点信息)
  - [ROS主题](#ros主题)
  - [ROS服务](#ros服务)
  - [消息定义](#消息定义)
  - [使用方法](#使用方法)
  - [ROS命令示例](#ros命令示例)
- [English Documentation](#english-documentation)
  - [Description](#description)
  - [System Requirements](#system-requirements)
  - [Installation & Build](#installation--build)
  - [ROS Node Information](#ros-node-information)
  - [ROS Topics](#ros-topics)
  - [ROS Services](#ros-services)
  - [Message Definitions](#message-definitions)
  - [Usage](#usage)
  - [ROS Command Examples](#ros-command-examples)

---

# 中文说明

## 功能描述

`forklift_drive` 是深圳市润木机器人有限公司开发的叉车驱动ROS包，主要功能包括：

- 🚗 叉车驱动电机控制（前进/后退/转向）
- 🔧 货叉升降控制
- 💡 三色灯和蜂鸣器控制
- 🔋 充电系统管理
- 📡 传感器数据采集（超声波雷达、激光雷达、碰撞传感器等）
- 🎵 MP3音频播放控制
- 🔌 串口通信
- 🛡️ 智能避障功能（激光雷达+超声波雷达多重保护）
- 🔧 翼板电机控制（自动开合货物挡板）
- ⚠️ 警示灯控制（多种工作状态指示）

## 系统要求

- **操作系统**: Ubuntu 18.04 / 20.04
- **ROS版本**: Melodic / Noetic
- **依赖包**:
  - `roscpp`
  - `rospy`
  - `std_msgs`
  - `geometry_msgs`
  - `message_generation`
  - `message_runtime`
  - `serial` (第三方串口库)

## 安装与编译

### 1. 克隆代码到工作空间

```bash
cd ~/catkin_ws/src
sudo chmod 777 -R forklift_drive.rar
unrar forklift_drive
```

### 2. 安装依赖

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译

```bash
cd ~/catkin_ws
catkin_make
```

### 4. 设置环境变量

```bash
source ~/catkin_ws/devel/setup.bash
```

## ROS节点信息

### 主节点: `forklift_drive_node`

- **节点名称**: `forklift_drive_node`
- **可执行文件**: `forklift_drive`
- **运行频率**: 20Hz
- **功能**: 叉车硬件接口驱动，负责串口通信和数据转换

## ROS主题

### 发布的主题 (Published Topics)

| 主题名称 | 消息类型 | 频率 | 描述 |
|---------|---------|------|------|
| `forklift_drive_publisher` | `forklift_drive/modbus_server` | 20Hz | 发布叉车状态信息（传感器数据、电机状态、电池信息等） |

### 订阅的主题 (Subscribed Topics)

| 主题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `forklift_drive_subscriber` | `forklift_drive/modbus_client` | 接收叉车控制命令（速度、转向、升降等） |

## ROS服务

| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `charging` | `forklift_drive/chargingCmd` | 充电控制服务 |

## 消息定义

### modbus_client.msg (控制命令消息)

```bash
# 查看消息定义
rosmsg show forklift_drive/modbus_client
```

**字段说明**:
- `driveSpeed` (int16): 驱动速度 (r/min)，正值前进，负值后退
- `liftControl` (int8): 货叉控制，0=停止，1=上升，2=下降
- `steerAngle` (int16): 转向角度 (0.01°)，正值右转，负值左转
- `lightRed/Green/Yellow` (uint8): 三色灯控制，1=开启，0=关闭
- `leftObstacleLidarMap/rightObstacleLidarMap` (uint8): 避障激光雷达地图映射 (1-8)
- `sensorRecharge` (uint8): 自动充电红外传感器，1=开启
- `sensorCharge` (uint8): 充电接触器，1=开启
- `mp3Channel` (uint8): MP3播放通道 (1-7)
- `buzzer` (uint8): 蜂鸣器控制，1=开启
- `wingsMotorControl` (uint8): 翼板电机控制，0=停止，1=打开，2=关闭
- `warningLightControl` (uint8): 警示灯控制，0=关闭，1=开启

**避障功能相关字段**:
- `leftObstacleLidarMap/rightObstacleLidarMap` (uint8): 激光雷达避障地图设置，用于配置不同检测区域的敏感度
- 配合超声波雷达实现多层次避障保护，确保叉车安全运行

### modbus_server.msg (状态反馈消息)

```bash
# 查看消息定义
rosmsg show forklift_drive/modbus_server
```

**主要字段**:
- **电机状态**: `driveSpeed`, `steerAngle`, `liftControl`
- **安全传感器**: `emergencyStop`, `bumper`, `buttonStop/Start`
- **叉尖避障传感器**: `leftForkTipAvoid`, `rightForkTipAvoid`
- **上下限位开关**: `forkUpLimitStatus`, `forkDownLimitStatus`
- **货物检测**: `cargoStatus`
- **充电状态**: `chargeStatus`
- **激光雷达触发区域**: `leftObstacleLidarTrigger1/2/3`, `rightObstacleLidarTrigger1/2/3`
- **电池信息**: `batteryPctg` (0-100%), `batteryVal` (V), `batteryCurrent` (0.1A)
- **超声波雷达**: 多个方向的距离数据 (mm)
- **编码器**: `dirveMotorEncoder`
- **翼板状态**: `wingsStatus` - 翼板当前位置状态，bit0=上升状态，bit1=下降状态

**避障系统详细说明**:
- **激光雷达**: 3个检测区域（左右各3个），可根据不同的路宽度，配置不同敏感度地图：1-8种（地图使用专门的上位机软件生成，自定义地图可以自由绘制）
- **超声波雷达**: 6个方向检测（左上、左中、左下、右上、右中、右下）
- **叉尖避障**: 专门的叉尖障碍物检测传感器
- **多重保护**: 激光雷达+超声波雷达+叉尖传感器三重避障保护

## 使用方法

### 1. 启动节点

```bash
# 使用launch文件启动
roslaunch forklift_drive forklift_drive.launch

# 或者直接运行节点
rosrun forklift_drive forklift_drive
```

### 2. 配置串口参数

在launch文件中设置串口参数：

```xml
<launch>
  <node name="forklift_drive_node" pkg="forklift_drive" type="forklift_drive" output="screen">
    <param name="port" value="/dev/ttyUSB0" />
    <param name="baudrate" value="115200" />
  </node>
</launch>
```

## ROS命令示例

### 查看节点信息

```bash
# 查看运行的节点
rosnode list

# 查看节点详细信息
rosnode info /forklift_drive_node

# 查看节点发布和订阅的主题
rostopic list | grep forklift_drive
```

### 主题操作

```bash
# 查看主题信息
rostopic info forklift_drive_publisher
rostopic info forklift_drive_subscriber

# 实时查看主题数据
rostopic echo forklift_drive_publisher

# 发布控制命令（示例：前进）
rostopic pub forklift_drive_subscriber forklift_drive/modbus_client "
header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
driveSpeed: 100
liftControl: 0
steerAngle: 0
lightRed: 0
lightGreen: 1
lightYellow: 0
leftObstacleLidarMap: 0
rightObstacleLidarMap: 0
sensorRecharge: 0
sensorCharge: 0
mp3Channel: 0
buzzer: 0
isDebug: false
wingsMotorControl: 0
warningLightControl: 0"

# 查看主题发布频率
rostopic hz forklift_drive_publisher
```

### 服务操作

```bash
# 查看可用服务
rosservice list | grep forklift_drive

# 查看服务定义
rosservice type charging
rossrv show forklift_drive/chargingCmd

# 调用充电服务（开始充电）
rosservice call charging "chargingCmd: 1"
```

### 消息和服务定义查看

```bash
# 查看所有自定义消息
rosmsg package forklift_drive

# 查看所有自定义服务
rossrv package forklift_drive

# 查看具体消息结构
rosmsg show forklift_drive/modbus_client
rosmsg show forklift_drive/modbus_server

# 查看具体服务结构
rossrv show forklift_drive/chargingCmd
rossrv show forklift_drive/chargingStatus
```

### 调试和监控

```bash
# 使用rqt工具监控
rqt_graph  # 查看节点和主题关系图
rqt_plot   # 绘制数据曲线
rqt_topic  # 主题监控器

# 记录和回放数据
rosbag record forklift_drive_publisher forklift_drive_subscriber
rosbag play <bagfile>

# 查看系统状态
rostopic echo /rosout | grep forklift_drive
```

---

# English Documentation

## Description

`forklift_drive` is a ROS package developed by Shenzhen Raymond Robotics Co., Ltd. for forklift control and monitoring. Main features include:

- 🚗 Forklift drive motor control (forward/backward/steering)
- 🔧 Fork lift control
- 💡 Tricolor light and buzzer control
- 🔋 Charging system management
- 📡 Sensor data acquisition (ultrasonic radar, lidar, collision sensors, etc.)
- 🎵 MP3 audio playback control
- 🔌 Modbus serial communication
- 🛡️ Intelligent obstacle avoidance (LiDAR + ultrasonic radar multi-layer protection)
- 🔧 Wings motor control (automatic cargo baffle opening/closing)
- ⚠️ Warning light control (multiple working status indication)

## System Requirements

- **OS**: Ubuntu 18.04 / 20.04
- **ROS Version**: Melodic / Noetic
- **Dependencies**:
  - `roscpp`
  - `rospy`
  - `std_msgs`
  - `geometry_msgs`
  - `message_generation`
  - `message_runtime`
  - `serial` (third-party serial library)

## Installation & Build

### 1. Clone to workspace

```bash
cd ~/catkin_ws/src
git clone <repository_url> forklift_drive
```

### 2. Install dependencies

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
cd ~/catkin_ws
catkin_make
# or use catkin build
catkin build forklift_drive
```

### 4. Source environment

```bash
source ~/catkin_ws/devel/setup.bash
```

## ROS Node Information

### Main Node: `forklift_drive_node`

- **Node Name**: `forklift_drive_node`
- **Executable**: `forklift_drive`
- **Running Frequency**: 20Hz
- **Function**: Forklift hardware interface driver, responsible for serial communication and data conversion

## ROS Topics

### Published Topics

| Topic Name | Message Type | Frequency | Description |
|-----------|-------------|-----------|-------------|
| `forklift_drive_publisher` | `forklift_drive/modbus_server` | 20Hz | Publishes forklift status information (sensor data, motor status, battery info, etc.) |

### Subscribed Topics

| Topic Name | Message Type | Description |
|-----------|-------------|-------------|
| `forklift_drive_subscriber` | `forklift_drive/modbus_client` | Receives forklift control commands (speed, steering, lifting, etc.) |

## ROS Services

| Service Name | Service Type | Description |
|-------------|-------------|-------------|
| `charging` | `forklift_drive/chargingCmd` | Charging control service |

## Message Definitions

### modbus_client.msg (Control Command Message)

```bash
# View message definition
rosmsg show forklift_drive/modbus_client
```

**Field Descriptions**:
- `driveSpeed` (int16): Drive speed (r/min), positive: forward, negative: backward
- `liftControl` (int8): Fork control, 0=stop, 1=up, 2=down
- `steerAngle` (int16): Steering angle (0.01°), positive: right, negative: left
- `lightRed/Green/Yellow` (uint8): Tricolor light control, 1=on, 0=off
- `leftObstacleLidarMap/rightObstacleLidarMap` (uint8): Obstacle avoidance lidar map (1-8)
- `sensorRecharge` (uint8): Auto charging IR sensor, 1=on
- `sensorCharge` (uint8): Charging contactor, 1=on
- `mp3Channel` (uint8): MP3 playback channel (1-7)
- `buzzer` (uint8): Buzzer control, 1=on
- `wingsMotorControl` (uint8): Wings motor control, 0=stop, 1=open, 2=close
- `warningLightControl` (uint8): Warning light control, 0=off, 1=on

**Obstacle Avoidance Related Fields**:
- `leftObstacleLidarMap/rightObstacleLidarMap` (uint8): LiDAR obstacle avoidance map settings for configuring sensitivity of different detection zones
- Works with ultrasonic radar to provide multi-layer obstacle protection for safe forklift operation

### modbus_server.msg (Status Feedback Message)

```bash
# View message definition
rosmsg show forklift_drive/modbus_server
```

**Main Fields**:
- **Motor Status**: `driveSpeed`, `steerAngle`, `liftControl`
- **Safety Sensors**: `emergencyStop`, `bumper`, `buttonStop/Start`
- **Obstacle Avoidance**: `leftForkTipAvoid`, `rightForkTipAvoid`
- **Limit Switches**: `forkUpLimitStatus`, `forkDownLimitStatus`
- **Cargo Detection**: `cargoStatus`
- **Charging Status**: `chargeStatus`
- **Lidar Triggers**: `leftObstacleLidarTrigger1/2/3`, `rightObstacleLidarTrigger1/2/3`
- **Battery Info**: `batteryPctg` (0-100%), `batteryVal` (V), `batteryCurrent` (0.1A)
- **Ultrasonic Radar**: Multi-directional distance data (mm)
- **Encoder**: `dirveMotorEncoder`
- **Wings Status**: `wingsStatus` - Current wings position, bit0=rise status, bit1=fall status

**Obstacle Avoidance System Details**:
- **LiDAR**: 3 detection zones (3 on each side), configurable sensitivity maps 1-8 based on different road widths (maps generated by dedicated PC software, custom maps can be freely drawn)
- **Ultrasonic Radar**: 6-directional detection (left-up, left-middle, left-down, right-up, right-middle, right-down)
- **Fork Tip Avoidance**: Dedicated fork tip obstacle detection sensors
- **Multi-layer Protection**: Triple protection with LiDAR + ultrasonic radar + fork tip sensors

## Usage

### 1. Launch Node

```bash
# Launch using launch file
roslaunch forklift_drive forklift_drive.launch

# Or run node directly
rosrun forklift_drive forklift_drive
```

### 2. Configure Serial Parameters

Set serial parameters in launch file:

```xml
<launch>
  <node name="forklift_drive_node" pkg="forklift_drive" type="forklift_drive" output="screen">
    <param name="port" value="/dev/ttyUSB0" />
    <param name="baudrate" value="115200" />
  </node>
</launch>
```

## ROS Command Examples

### Node Information

```bash
# List running nodes
rosnode list

# Show detailed node information
rosnode info /forklift_drive_node

# Show topics published/subscribed by node
rostopic list | grep forklift_drive
```

### Topic Operations

```bash
# Show topic information
rostopic info forklift_drive_publisher
rostopic info forklift_drive_subscriber

# Monitor topic data in real-time
rostopic echo forklift_drive_publisher

# Publish control command (example: move forward)
rostopic pub forklift_drive_subscriber forklift_drive/modbus_client "
header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
driveSpeed: 100
liftControl: 0
steerAngle: 0
lightRed: 0
lightGreen: 1
lightYellow: 0
leftObstacleLidarMap: 0
rightObstacleLidarMap: 0
sensorRecharge: 0
sensorCharge: 0
mp3Channel: 0
buzzer: 0
isDebug: false
wingsMotorControl: 0
warningLightControl: 0"

# Check topic publishing frequency
rostopic hz forklift_drive_publisher
```

### Service Operations

```bash
# List available services
rosservice list | grep charging

# Show service definition
rosservice type charging
rossrv show forklift_drive/chargingCmd

# Call charging service (start charging)
rosservice call charging "chargingCmd: 1"
```

### Message and Service Definition Viewing

```bash
# Show all custom messages
rosmsg package forklift_drive

# Show all custom services
rossrv package forklift_drive

# Show specific message structure
rosmsg show forklift_drive/modbus_client
rosmsg show forklift_drive/modbus_server

# Show specific service structure
rossrv show forklift_drive/chargingCmd
rossrv show forklift_drive/chargingStatus
```

### Debugging and Monitoring

```bash
# Use rqt tools for monitoring
rqt_graph  # View node and topic relationship graph
rqt_plot   # Plot data curves
rqt_topic  # Topic monitor

# Record and playback data
rosbag record forklift_drive_publisher forklift_drive_subscriber
rosbag play <bagfile>

# Check system status
rostopic echo /rosout | grep forklift_drive
```

---

## 📞 技术支持 | Technical Support

- **公司**: 深圳市润木机器人有限公司 | Shenzhen Raymond Robotics Co., Ltd.
- **邮箱**: loveredhome@126.com
- **版本**: v1.0.1

## 📄 许可证 | License

本软件为深圳市润木机器人有限公司专有软件，未经授权不得使用、复制或传播。

This software is proprietary to Shenzhen Raymond Robotics Co., Ltd. Unauthorized use, copying, or distribution is prohibited.