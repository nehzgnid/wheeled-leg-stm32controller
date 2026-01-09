# STM32F407 多功能嵌入式控制系统

## 项目概述

这是一个基于STM32F407VET6的多功能嵌入式控制系统，集成了GPIO控制、IMU传感器读取、PCA9685舵机驱动和UART通信功能。项目使用STM32CubeMX生成基础框架，配合Keil MDK-ARM开发环境，采用FreeRTOS实现多任务并发执行。

### 主要功能模块

1. **GPIO控制模块**: 控制PB12引脚输出
2. **IMU传感器模块**: 通过I2C1接口读取MPU6050六轴传感器数据
3. **PCA9685舵机驱动模块**: 通过I2C2接口控制16路舵机
4. **UART通信模块**: 通过UART4和UART5实现与上位机的双向通信

### 通信协议

项目实现了新的多舵机控制消息机制：

- **舵机控制消息格式**: `SERVO,<servo_id>,<angle>,<duration>\n`
  - 例如: `SERVO,0,90,1000\n` - 控制0号舵机转到90度，用时1000ms
- **姿态回传消息格式**: `POSE,<angle0>,<angle1>,...,<angle15>,<timestamp>,<accel_x>,<accel_y>,<accel_z>,<gyro_x>,<gyro_y>,<gyro_z>,<temperature>\n`
  - 以100Hz频率回传当前所有舵机的姿态信息和IMU数据

## 构建和运行

### 开发环境
- STM32CubeMX 6.x
- Keil MDK-ARM (或兼容IDE)
- STM32F4 HAL库

### 项目结构
```
GPIO_Test_F4/
├── Core/
│   ├── Inc/          # 头文件
│   │   ├── jetson_uart.h
│   │   ├── main.h
│   │   ├── bsp_mpu6050.h
│   │   ├── pca9685.h
│   │   └── ...
│   └── Src/          # 源文件
│       ├── jetson_uart.c
│       ├── main.c
│       ├── bsp_mpu6050.c
│       ├── pca9685.c
│       └── ...
├── Drivers/          # HAL驱动
├── MDK-ARM/          # Keil项目文件
└── README.md
```

### 编译步骤
1. 使用STM32CubeMX打开GPIO_Test_F4.ioc文件
2. 生成代码
3. 使用Keil MDK-ARM打开生成的项目
4. 编译项目

### 硬件连接
- MPU6050: VCC->3.3V, GND->GND, SCL->PB6, SDA->PB7
- PCA9685: VCC->5V, GND->GND, SCL->PB10, SDA->PB11
- 舵机: 连接到PCA9685的OUT0-OUT15输出端口
- 串口: 连接到PC或其他串口设备

## 开发约定

### 代码风格
- 遏循ST官方HAL库编程规范
- 使用C语言编写
- 采用FreeRTOS任务调度机制
- 使用Doxygen风格的函数注释

### 任务优先级
- GPIO_Task: osPriorityNormal
- UART_Task: osPriorityLow
- Imu_Task: osPriorityLow
- PCA9685_Task: osPriorityLow
- Jetson_Task: osPriorityLow

### 通信协议设计
- UART5用于与上位机的通信
- 支持标准行结束符(\n, \r\n, \r)
- 实现了舵机控制和姿态回传功能
- 优化了任务分工，PCA9685_Task只负责设备状态监控，舵机控制由UART接收消息驱动

## 关键特性

1. **多任务并发**: 四个独立任务同时运行，互不干扰
2. **独立通信**: I2C1和I2C2完全独立，避免总线冲突
3. **实时响应**: 传感器数据实时更新，舵机平滑控制
4. **状态监控**: 通过UART实时输出系统运行状态
5. **扩展性强**: 模块化设计，便于功能扩展
6. **高效通信**: 实现了100Hz频率的舵机控制和姿态回传
7. **统一数据结构**: 综合姿态信息结构体整合舵机和IMU数据

## 应用场景

适用于轮式腿式机器人控制器、多自由度机械臂控制、姿态感知系统等应用场景。