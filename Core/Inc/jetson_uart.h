/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_uart.h
  * @brief          : Header for jetson_uart.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __JETSON_UART_H
#define __JETSON_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// 定义舵机控制消息结构体
typedef struct {
    uint8_t servo_channel;    // 舵机通道 (0-15)
    uint8_t target_angle;     // 目标角度 (0-180)
    uint8_t speed;            // 速度 (1-9, 数值越大越慢)
} ServoControlMsg_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
// 解析命令函数
uint8_t parse_servo_command(uint8_t *buf, uint8_t len, ServoControlMsg_t *msg);

// 发送应答消息
void send_ack_message(uint8_t channel, uint8_t angle);

// 发送错误消息
void send_error_message(const char* error_msg);

// UART5接收相关全局变量声明
extern uint8_t uart5_rx_data;
extern uint8_t uart5_rx_buffer[100];
extern uint8_t uart5_rx_index;
extern uint8_t uart5_frame_complete;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __JETSON_UART_H */