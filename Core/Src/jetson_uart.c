/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_uart.c
  * @brief          : UART communication implementation for Jetson Nano
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

/* Includes ------------------------------------------------------------------*/
#include "jetson_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// UART5接收缓冲区和索引
uint8_t uart5_rx_data;
uint8_t uart5_rx_buffer[100];
uint8_t uart5_rx_index = 0;
uint8_t uart5_frame_complete = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Parse servo command from received data
  * @param  buf: Received data buffer
  * @param  len: Length of received data
  * @param  msg: Pointer to ServoControlMsg_t to store parsed data
  * @retval 1 if successful, 0 if failed
  */
uint8_t parse_servo_command(uint8_t *buf, uint8_t len, ServoControlMsg_t *msg) {
    // 检查命令格式是否正确 (#CH,ANGLE,SPEED*)
    if(len < 8 || buf[0] != '#' || buf[len-1] != '*') {
        return 0; // 格式错误
    }
    
    // 查找分隔符
    uint8_t *comma1 = NULL, *comma2 = NULL;
    for(int i = 1; i < len-1; i++) {
        if(buf[i] == ',') {
            if(comma1 == NULL) {
                comma1 = &buf[i];
            } else if(comma2 == NULL) {
                comma2 = &buf[i];
                break;
            }
        }
    }
    
    if(comma1 == NULL || comma2 == NULL) {
        return 0; // 分隔符不足
    }
    
    // 提取参数
    char channel_str[4], angle_str[4], speed_str[4];
    
    // 提取通道号
    int ch_len = comma1 - &buf[1];
    if(ch_len > 3) ch_len = 3;
    strncpy(channel_str, (char*)&buf[1], ch_len);
    channel_str[ch_len] = '\0';
    
    // 提取角度
    int angle_len = comma2 - (comma1 + 1);
    if(angle_len > 3) angle_len = 3;
    strncpy(angle_str, (char*)(comma1 + 1), angle_len);
    angle_str[angle_len] = '\0';
    
    // 提取速度
    int speed_len = &buf[len-1] - (comma2 + 1);
    if(speed_len > 3) speed_len = 3;
    strncpy(speed_str, (char*)(comma2 + 1), speed_len);
    speed_str[speed_len] = '\0';
    
    // 转换字符串为数值
    int channel = atoi(channel_str);
    int angle = atoi(angle_str);
    int speed = atoi(speed_str);
    
    // 参数范围检查
    if(channel < 0 || channel > 15 || angle < 0 || angle > 180 || speed < 1 || speed > 9) {
        return 0; // 参数超出范围
    }
    
    msg->servo_channel = (uint8_t)channel;
    msg->target_angle = (uint8_t)angle;
    msg->speed = (uint8_t)speed;
    
    return 1; // 成功解析
}

/**
  * @brief  Send acknowledgment message to Jetson
  * @param  channel: Servo channel
  * @param  angle: Target angle
  * @retval None
  */
void send_ack_message(uint8_t channel, uint8_t angle) {
    char ack_msg[50];
    sprintf(ack_msg, "!ACK,%02d,%d*\r\n", channel, angle);
    HAL_UART_Transmit(&huart5, (uint8_t*)ack_msg, strlen(ack_msg), 0xFFFF);
}

/**
  * @brief  Send error message to Jetson
  * @param  error_msg: Error message string
  * @retval None
  */
void send_error_message(const char* error_msg) {
    char err_response[50];
    sprintf(err_response, "!ERR,%s*\r\n", error_msg);
    HAL_UART_Transmit(&huart5, (uint8_t*)err_response, strlen(err_response), 0xFFFF);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */