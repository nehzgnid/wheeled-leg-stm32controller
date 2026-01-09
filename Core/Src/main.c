/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_mpu6050.h"
#include "pca9685.h"
#include "jetson_uart.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* Definitions for gpiotask */
osThreadId_t gpiotaskHandle;
const osThreadAttr_t gpiotask_attributes = {
  .name = "gpiotask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart_task */
osThreadId_t uart_taskHandle;
const osThreadAttr_t uart_task_attributes = {
  .name = "uart_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imu_task */
osThreadId_t imu_taskHandle;
const osThreadAttr_t imu_task_attributes = {
  .name = "imu_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/* Definitions for pca9685task */
osThreadId_t pca9685taskHandle;
const osThreadAttr_t pca9685task_attributes = {
  .name = "pca9685task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for jetson_task */
osThreadId_t jetson_taskHandle;
const osThreadAttr_t jetson_task_attributes = {
  .name = "jetson_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART5_Init(void);
void GPIO_Task(void *argument);
void UART_Task(void *argument);
void Imu_Task(void *argument);
void PCA9685_Task(void *argument);
void Jetson_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MX_UART4_Init();

  // 系统启动信息输出
  char start_msg[] = "System Started Successfully!\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)start_msg, strlen(start_msg), 0xFFFF);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of gpiotask */
  gpiotaskHandle = osThreadNew(GPIO_Task, NULL, &gpiotask_attributes);

  /* creation of uart_task */
  uart_taskHandle = osThreadNew(UART_Task, NULL, &uart_task_attributes);

  /* creation of imu_task */
  imu_taskHandle = osThreadNew(Imu_Task, NULL, &imu_task_attributes);

  /* creation of pca9685task */
  pca9685taskHandle = osThreadNew(PCA9685_Task, NULL, &pca9685task_attributes);

  /* creation of jetson_task */
  jetson_taskHandle = osThreadNew(Jetson_Task, NULL, &jetson_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  // 使能UART5中断，调整优先级以避免与系统中断冲突
  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/* USER CODE BEGIN Header_GPIO_Task */
/**
  * @brief  Function implementing the gpiotask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_GPIO_Task */
void GPIO_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UART_Task */
/**
* @brief Function implementing the uart_task thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE BEGIN Header_UART_Task */
/**
* @brief Function implementing the uart_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Task */
void UART_Task(void *argument)
{
  /* USER CODE BEGIN UART_Task */
  uint32_t cnt = 0;
  char uart_send_buf[50];

  // 等待系统稳定
  osDelay(1000);

  while(1)
  {
      cnt++;
//      sprintf(uart_send_buf, "UART Message Count: %lu\r\n", cnt);

//      // 发送数据到串口
//      HAL_UART_Transmit(&huart4, (uint8_t*)uart_send_buf, strlen(uart_send_buf), 0xFFFF);

      // 每隔1秒发送一次
      osDelay(1000);
  }
  /* USER CODE END UART_Task */
}

/* USER CODE BEGIN Header_Imu_Task */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Imu_Task */
void Imu_Task(void *argument)
{
  /* USER CODE BEGIN Imu_Task */
  MPU6050_Data_t imu_data;
  char imu_send_buf[100];
  HAL_StatusTypeDef status;

  // 等待系统稳定
  osDelay(1000);

  // 初始化MPU6050传感器
  status = BSP_MPU6050_Init(&hi2c1);
  if(status != HAL_OK)
  {
      // 初始化失败，发送错误信息
      sprintf(imu_send_buf, "MPU6050 Initialization Failed!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
  }
  else
  {
      // 初始化成功，发送确认信息
      sprintf(imu_send_buf, "MPU6050 Initialization Success!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
  }

  /* Infinite loop */
  for(;;)
  {
    // 读取MPU6050传感器数据
    status = BSP_MPU6050_Read_Data(&hi2c1, &imu_data);

    if(status == HAL_OK)
    {
        // 格式化并发送IMU数据
        sprintf(imu_send_buf,
                "IMU: Accel[X:%.2fg,Y:%.2fg,Z:%.2fg] Gyro[X:%.2fdps,Y:%.2fdps,Z:%.2fdps] Temp:%.2fc\r\n",
                imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g,
                imu_data.gyro_x_dps, imu_data.gyro_y_dps, imu_data.gyro_z_dps,
                imu_data.temperature_c);

        // 更新IMU数据到综合姿态信息
        extern void UpdateIMUData(float accel_x, float accel_y, float accel_z,
                                  float gyro_x, float gyro_y, float gyro_z,
                                  float temperature);
        UpdateIMUData(imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g,
                      imu_data.gyro_x_dps, imu_data.gyro_y_dps, imu_data.gyro_z_dps,
                      imu_data.temperature_c);
    }
    else
    {
        sprintf(imu_send_buf, "MPU6050 Read Error!\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
    }

    // 每隔100ms读取一次数据 (10Hz)
    osDelay(100);
  }
  /* USER CODE END Imu_Task */
}

/* USER CODE BEGIN Header_PCA9685_Task */
/**
* @brief Function implementing the pca9685task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PCA9685_Task */
void PCA9685_Task(void *argument)
{
  /* USER CODE BEGIN PCA9685_Task */
  char servo_send_buf[100];
  HAL_StatusTypeDef status;

  // 等待系统稳定
  osDelay(1000);

  // 初始化PCA9685，设置PWM频率为50Hz（舵机标准频率）
  status = HAL_I2C_IsDeviceReady(&hi2c2, PCA9685_ADDR, 5, 100);
  if(status == HAL_OK)
  {
      PCA9685_Init(&hi2c2, 50.0f);  // 50Hz for servo control

      // 发送初始化成功信息
      sprintf(servo_send_buf, "PCA9685 Initialization Success!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)servo_send_buf, strlen(servo_send_buf), 0xFFFF);
  }
  else
  {
      // 发送初始化失败信息
      sprintf(servo_send_buf, "PCA9685 Device Not Found!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)servo_send_buf, strlen(servo_send_buf), 0xFFFF);
  }

  /* Infinite loop */
  for(;;)
  {
      // 现在舵机控制由UART接收消息驱动，这里只负责检查设备状态
      if(status != HAL_OK)
      {
          // 如果设备未找到，尝试重新初始化
          status = HAL_I2C_IsDeviceReady(&hi2c2, PCA9685_ADDR, 5, 100);
          if(status == HAL_OK)
          {
              PCA9685_Init(&hi2c2, 50.0f);  // 50Hz for servo control

              // 发送重新初始化成功信息
              sprintf(servo_send_buf, "PCA9685 Re-initialization Success!\r\n");
              HAL_UART_Transmit(&huart4, (uint8_t*)servo_send_buf, strlen(servo_send_buf), 0xFFFF);
          }
      }

      // 延迟较长时间，因为主要控制逻辑在UART接收部分
      osDelay(1000);
  }
  /* USER CODE END PCA9685_Task */
}

/* USER CODE BEGIN Header_Jetson_Task */
/**
* @brief Function implementing the jetson_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Jetson_Task */
void Jetson_Task(void *argument)
{
  /* USER CODE BEGIN Jetson_Task */
  char debug_msg[100];

  // 用于100Hz姿态回传的时间跟踪
  uint32_t last_pose_transmit = 0;
  const uint32_t POSE_TRANSMIT_INTERVAL = 10; // 10ms = 100Hz

  // 初始化UART通信
  JetsonUart_Init();

  // 等待UART完全初始化
  osDelay(100);

  // 启动UART5中断接收
  if(JetsonUart_StartReceiveIT() != HAL_OK) {
      sprintf(debug_msg, "Failed to start UART5 IT receive\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
  } else {
      sprintf(debug_msg, "UART5 IT receive started successfully\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
  }

  // 等待系统稳定
  osDelay(1000);

  // 发送初始化完成消息
  sprintf(debug_msg, "Jetson Comm Task Started at 100Hz\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);

  /* Infinite loop */
  for(;;)
  {
    // 100Hz通信速率，即每10ms执行一次循环
    // 数据接收由中断处理，这里可以做其他事情
    osDelay(10);

    // 检查接收超时并处理
    Jetson_CheckReceiveTimeout();
    // 定期清理可能的陈旧数据
    Jetson_ClearStaleData();

    // 检查姿态是否发生变化，如果变化则发送
    if(CheckPoseChanged()) {
        // 发送当前姿态
        SendCurrentPose();
        last_pose_transmit = HAL_GetTick();
    }
    // 同时保持最小发送间隔，避免过于频繁发送
    else if((HAL_GetTick() - last_pose_transmit) >= POSE_TRANSMIT_INTERVAL) {
        // 即使没有变化，也定期发送以维持通信
        SendCurrentPose();
        last_pose_transmit = HAL_GetTick();
    }
  }
  /* USER CODE END Jetson_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  char error_msg[] = "Error_Handler: System Error Occurred!\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 0xFFFF);

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
