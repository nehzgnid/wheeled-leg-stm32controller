# 代码结构与系统架构深度解析

经过对项目源代码的完整阅读与分析，我为您整理了当前系统的代码结构、模块关系及核心逻辑流程。这是一个基于 **FreeRTOS** 的典型嵌入式实时控制系统。

## 1. 核心文件架构

项目代码主要分布在 `Core/Src` (源文件) 和 `Core/Inc` (头文件) 中，按功能可划分为以下层级：

### A. 系统与调度层 (System & OS)
*   **`main.c`**: 系统的心脏。
    *   **职责**: 硬件初始化 (HAL_Init, SystemClock, GPIO, UART, I2C, TIM)、FreeRTOS 任务创建 (`osThreadNew`)、队列/互斥锁创建。
    *   **关键点**: 定义了所有任务的入口函数（如 `Jetson_Task`, `MotorSpeedTask` 等）。
*   **`freertos.c` / `app_freertos.c`**: FreeRTOS 的配置钩子文件，当前主要逻辑已移至 `main.c`。
*   **`stm32f4xx_it.c`**: 中断服务函数 (ISR)。
    *   **关键点**: `UART5_IRQHandler` 负责处理上位机通信中断，`TIM4_IRQHandler` 提供系统心跳。

### B. 通信协议层 (Protocol)
*   **`jetson_uart.c` / `.h`**: 与上位机 (Jetson Nano) 的通信核心。
    *   **职责**:
        *   **接收**: 实现 UART5 中断接收 (`HAL_UART_RxCpltCallback`)，解析 `SERVO`, `BATCH_SERVO`, `MOTOR` 指令。
        *   **发送**: 打包电机、舵机、IMU 数据，通过 `SendCurrentPose` 发送 `POSE` 协议包。
        *   **数据共享**: 维护全局状态结构体 `comprehensive_pose`，汇聚各模块数据。

### C. 硬件驱动层 (Drivers)
*   **`Motor.c` / `.h`**: 直流电机参数定义。
    *   **内容**: 定义减速比 (`GEAR_RATIO`), 编码器倍频 (`ENCODER_MULTIPLIER`), 速度采样时间等。
*   **`pca9685.c` / `.h`**: 16路 PWM 舵机驱动。
    *   **功能**: I2C 通信配置，PWM 频率设置，`PCA9685_SetAngle` 实现角度控制，以及平滑移动逻辑。
*   **`bsp_mpu6050.c` / `.h`**: IMU 传感器驱动。
    *   **功能**: I2C 寄存器读写，原始数据转换为物理量 (g, dps, ℃)。

---

## 2. 核心逻辑数据流

系统通过 **全局变量 + 互斥锁** 以及 **FreeRTOS 任务** 实现数据流转。

### A. 指令下发流程 (上位机 -> 执行器)
1.  **接收**: 上位机发送指令 (e.g., `MOTOR,0,3,50`) -> **UART5 中断** -> `jetson_uart.c` 缓冲区。
2.  **解析**: `Jetson_Task` 或中断回调调用解析函数 (`ParseMotorControlMessage`)。
3.  **分发**:
    *   **电机**: 解析后的指令 (方向, 速度) 存入 `motor_cmd_direction` / `motor_cmd_speed` 全局数组 (受 `motor_cmd_mutex` 保护)。
    *   **舵机**:
        *   立即指令: 直接调用 `PCA9685_SetAngle`。
        *   平滑指令: 更新 `servo_motion` 结构体，由 `PCA9685_Task` 后续处理。

### B. 执行流程 (FreeRTOS 任务)
1.  **电机控制 (`MotorCtrl_TasK` x4)**:
    *   100Hz 频率运行。
    *   读取全局指令变量 -> 控制 GPIO (方向) -> 控制 PWM 占空比 (速度)。
    *   **关键**: 使用了不同的定时器通道 (TIM5, TIM9, TIM12) 驱动不同电机。
2.  **舵机执行 (`PCA9685_Task`)**:
    *   100Hz 频率运行。
    *   调用 `Servo_UpdateAll()`，检查 `servo_motion` 状态。
    *   如果需要移动，计算当前时间点的目标角度（线性插值），通过 I2C 更新 PCA9685。

### C. 状态回传流程 (传感器 -> 上位机)
1.  **数据采集**:
    *   **电机**: `MotorSpeedTask` (x4) 读取编码器 (TIM1,2,3,8)，计算 RPM，更新到 `all_motors` 数组。
    *   **IMU**: `Imu_Task` 读取 I2C1 数据，更新到 `comprehensive_pose.imu_data`。
    *   **舵机**: 记录当前目标角度。
2.  **打包发送**:
    *   `Jetson_Task` 以 100Hz 频率运行。
    *   汇聚所有数据 -> 调用 `SendCurrentPose` -> 格式化为 `POSE,...` 字符串 -> UART5 发送。

## 3. 关键配置参数
*   **PWM 频率**: 电机 PWM 周期设为 `13999` (约 1.5-2kHz)，舵机 PWM 设为 50Hz。
*   **通信波特率**: 115200 bps。
*   **电机参数**: 减速比 42.0，编码器线数 7，4倍频。

这份总结涵盖了您当前代码的所有核心结构。如果您需要修改某个具体环节（例如修改 PID 参数或增加新的指令），请告诉我，我可以精准定位到相关行。
