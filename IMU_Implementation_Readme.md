# IMU Task Implementation for STM32F4 Project

## Overview
This document describes the IMU (Inertial Measurement Unit) task implementation using the MPU6050 sensor. The implementation reads sensor data and sends it via UART every 100ms (10Hz).

## Hardware Configuration
- **Sensor**: MPU6050 (I2C address: 0x68)
- **I2C Interface**: I2C1
- **Pins Used**: 
  - PB6 (I2C1_SCL)
  - PB7 (I2C1_SDA)
- **Data Rate**: 10Hz (every 100ms)

## Software Components

### 1. I2C Initialization
- `MX_I2C1_Init()` - Configures I2C1 peripheral with 100kHz clock speed
- Standard 7-bit addressing mode
- Uses GPIO pins PB6 and PB7

### 2. IMU Task
- `Imu_Task()` - Implements the IMU data reading and transmission functionality
- Runs at low priority
- Performs sensor initialization and continuous data reading

### 3. Task Configuration
- IMU task runs at 2048-byte stack size (512*4 bytes)
- Priority set to `osPriorityLow`
- Uses FreeRTOS delay functions for timing

## Key Features

### Data Reading
- Reads accelerometer data (X, Y, Z axes) in g units
- Reads gyroscope data (X, Y, Z axes) in degrees per second
- Reads temperature data in Celsius

### Message Format
Each message sent follows the format:
```
IMU: Accel[X:val_g,Y:val_g,Z:val_g] Gyro[X:val_dps,Y:val_dps,Z:val_dps] Temp:val_c
```

Where:
- `val_g` = acceleration values in g units
- `val_dps` = gyroscope values in degrees per second
- `val_c` = temperature in Celsius

### Error Handling
- Checks for successful sensor initialization
- Reports read errors if they occur
- Uses HAL status codes for error detection

## Integration Points

### Main Function Changes
- Added I2C1 initialization in main initialization sequence
- Created IMU task using `osThreadNew()`
- Maintained existing UART and GPIO task functionality

### Memory Usage
- Added I2C_HandleTypeDef for I2C1
- Added task stack space (2048 bytes)
- Added data structures for IMU readings

## Usage Instructions

### Connecting to IMU
1. Connect MPU6050 sensor to I2C1 (PB6 - SCL, PB7 - SDA)
2. Ensure proper power supply (3.3V) to the sensor
3. Power on the board

### Expected Output
Every 100ms, you should see:
```
IMU: Accel[X:0.01g,Y:-0.02g,Z:1.00g] Gyro[X:0.15dps,Y:-0.23dps,Z:0.08dps] Temp:25.60c
```

## Code Structure

### Task Priority Hierarchy
1. `StartDefaultTask` - Default task (lowest priority)
2. `UART_Task` - UART communication task (low priority)
3. `Imu_Task` - IMU sensor task (low priority)
4. `GPIO_Task` - GPIO control task (normal priority)

### Resource Management
- I2C1 clock is properly enabled/disabled
- GPIO pins are configured for I2C alternate function
- MPU6050 sensor is initialized with ±2g accelerometer range and ±250°/s gyroscope range

## Troubleshooting

### Common Issues
1. **No IMU data output**:
   - Verify MPU6050 connections
   - Check I2C pull-up resistors (typically 4.7kΩ to 3.3V)
   - Confirm correct I2C address (0x68)

2. **Initialization failed message**:
   - Check sensor power supply
   - Verify I2C connections
   - Ensure sensor is not damaged

3. **Incorrect data values**:
   - Verify sensor orientation
   - Check for electrical interference

### Debugging Tips
- Use I2C scanner to verify sensor address
- Check I2C clock speed settings
- Monitor sensor power supply voltage

## Files Modified
- `main.c` - Added I2C initialization, IMU task implementation
- `bsp_mpu6050.c/h` - MPU6050 driver (already existed)

## Dependencies
- STM32F4xx HAL Library
- FreeRTOS
- CMSIS Core
- I2C1 peripheral availability
- MPU6050 sensor driver (bsp_mpu6050)