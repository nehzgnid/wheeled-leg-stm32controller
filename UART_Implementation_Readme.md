# UART Implementation for STM32F4 GPIO_Test_F4 Project

## Overview
This document describes the UART implementation added to the existing STM32F4 project. The implementation adds a UART task that sends a message every second via UART4.

## Hardware Configuration
- **Microcontroller**: STM32F407VET6
- **UART Interface**: UART4
- **Pins Used**: 
  - PA0 (UART4_TX)
  - PA1 (UART4_RX)
- **Baud Rate**: 115200
- **Data Format**: 8 bits, no parity, 1 stop bit

## Software Components

### 1. UART Initialization
- `MX_UART4_Init()` - Configures UART4 peripheral
- `HAL_UART_MspInit()` - Configures GPIO pins and interrupts
- `HAL_UART_MspDeInit()` - Deinitializes UART4 resources

### 2. UART Task
- `UART_Task()` - Implements the periodic message sending functionality
- Runs at normal priority
- Sends "UART Message Count: X" every second

### 3. Task Configuration
- UART task runs at 512-byte stack size
- Priority set to `osPriorityNormal`
- Uses FreeRTOS delay functions for timing

## Key Features

### Message Format
Each message sent follows the format:
```
UART Message Count: [counter_value]
```

Where `[counter_value]` is an incrementing integer starting from 1.

### Timing
- Initial delay of 1 second before first message
- Messages sent every 1000ms (1 second)
- Uses `osDelay()` for non-blocking delays

### Error Handling
- Uses HAL error checking during initialization
- Proper error handler integration

## Integration Points

### Main Function Changes
- Added `MX_UART4_Init()` call in initialization section
- Created UART task using `osThreadNew()`
- Maintained existing GPIO task functionality

### Memory Usage
- Added UART_HandleTypeDef for UART4
- Added task stack space (512 bytes)
- Added string buffer for message formatting

## Usage Instructions

### Connecting to UART
1. Connect UART4 TX (PA0) and RX (PA1) to your serial terminal
2. Configure terminal to 115200 baud, 8N1
3. Power on the board

### Expected Output
Every second, you should see:
```
UART Message Count: 1
UART Message Count: 2
UART Message Count: 3
...
```

## Code Structure

### Task Priority Hierarchy
1. `StartDefaultTask` - Default task (lowest priority)
2. `GPIO_Task` - GPIO control task (low priority) 
3. `UART_Task` - UART communication task (normal priority)

### Resource Management
- UART4 clock is properly enabled/disabled
- GPIO pins are configured as alternate function
- NVIC interrupts are configured for UART4
- Proper cleanup in MSP de-initialization

## Troubleshooting

### Common Issues
1. **No output on serial terminal**:
   - Verify wiring connections
   - Check baud rate settings (should be 115200)
   - Confirm correct TX/RX pin connections

2. **Garbled output**:
   - Check baud rate match between device and terminal
   - Verify ground connection is shared

3. **Task not running**:
   - Verify FreeRTOS scheduler is started
   - Check for stack overflow issues

### Debugging Tips
- Use SWD/JTAG for debugging if UART issues occur
- Check GPIO pin configurations in CubeMX if needed
- Monitor system clock configuration for UART accuracy

## Files Modified
- `main.c` - Added UART initialization, task implementation, and MSP functions
- Existing files remain unchanged to maintain compatibility

## Dependencies
- STM32F4xx HAL Library
- FreeRTOS
- CMSIS Core
- UART4 peripheral availability on target device