# 串口热插拔功能使用说明

## 功能概述

本叉车驱动模块现已支持串口热插拔功能，当串口设备断开连接时，系统会自动检测并尝试重新连接，无需重启程序。

## 主要特性

### 1. 自动重连机制
- **重连间隔**: 2秒
- **重连策略**: 无限重连（直到成功或程序退出）
- **智能端口检测**: 优先连接配置的端口，如果不可用则尝试其他可用端口

### 2. 连接状态监控
- 实时检测串口连接状态
- 自动处理连接丢失情况
- 错误数据检测和处理

### 3. 设备枚举
- 自动扫描系统中所有可用串口
- 支持动态端口分配
- 详细的端口信息日志

## 配置参数

在launch文件中可以配置以下参数：

```xml
<launch>
    <node name="forklift_drive" pkg="forklift_drive" type="forklift_drive_node">
        <!-- 串口配置 -->
        <param name="port" value="/dev/ttyUSB0" />
        <param name="baudrate" value="115200" />
    </node>
</launch>
```

## 使用方法

### 1. 正常启动
```bash
roslaunch forklift_drive forklift_drive.launch
```

### 2. 测试热插拔功能
```bash
# 运行测试程序
rosrun forklift_drive test_hotplug
```

### 3. 热插拔测试步骤
1. 启动程序后，确认串口连接正常
2. 物理断开串口设备（拔掉USB线）
3. 观察日志输出，系统会显示连接丢失信息
4. 重新连接串口设备（插入USB线）
5. 系统会自动检测并重新连接

## 日志信息说明

### 正常连接
```
[INFO] Serial Port initialized ok
[INFO] Hot-plug functionality initialized
```

### 连接丢失
```
[WARN] Serial connection lost, enabling hot-plug detection
```

### 重连尝试
```
[INFO] Attempting to reconnect serial port...
[INFO] Attempting to connect to configured port: /dev/ttyUSB0
```

### 重连成功
```
[INFO] Serial port reconnected successfully: /dev/ttyUSB0
```

### 端口扫描
```
[DEBUG] Found serial port: /dev/ttyUSB0 - USB Serial Device
[DEBUG] Found serial port: /dev/ttyUSB1 - USB Serial Device
```

## 故障排除

### 1. 无法找到串口设备
- 检查设备是否正确连接
- 确认设备驱动已安装
- 检查设备权限设置

### 2. 重连失败
- 检查串口是否被其他程序占用
- 确认波特率设置正确
- 查看系统日志获取详细错误信息

### 3. 频繁断连
- 检查USB线缆质量
- 确认电源供应稳定
- 检查设备硬件状态

## 技术实现

### 关键组件
- `initHotPlug()`: 初始化热插拔功能
- `isSerialConnected()`: 检测串口连接状态
- `attemptReconnect()`: 尝试重新连接
- `enumerateSerialPorts()`: 枚举可用串口
- `findAndConnectAvailablePort()`: 查找并连接可用端口
- `handleSerialDisconnection()`: 处理断连事件

### 工作流程
1. 程序启动时初始化热插拔功能
2. 在主循环中持续监控连接状态
3. 检测到断连时触发重连机制
4. 定时尝试重新连接直到成功

## 注意事项

1. **性能影响**: 热插拔检测会增加少量CPU开销，但对系统性能影响很小
2. **日志级别**: 建议在生产环境中适当调整日志级别以减少输出
3. **设备兼容性**: 功能已在常见USB转串口设备上测试，特殊设备可能需要额外配置
4. **线程安全**: 所有热插拔相关操作都在主线程中执行，确保线程安全

## 版本历史

- v1.0.1: 首次实现串口热插拔功能
  - 添加自动重连机制
  - 实现设备枚举功能
  - 集成连接状态监控
