/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_uart.c
  * @brief          : UART communication implementation (High-Speed Optimized)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------
*/
#include "jetson_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h" 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h" 
#include "pca9685.h"   
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint16_t angles[16];
    uint32_t timestamp;
} MultiServoPose_t;

typedef struct {
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float temperature_c;
} IMUData_t;

typedef struct {
    int8_t direction[4];
    float speed_rpm[4];
    uint32_t timestamp;
} MotorData_t;

typedef struct {
    MultiServoPose_t servo_pose;
    IMUData_t imu_data;
    MotorData_t motor_data;
    uint32_t timestamp;
} ComprehensivePose_t;

/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

#define RX_RING_BUFFER_SIZE 2048
static uint8_t rx_ring_buf[RX_RING_BUFFER_SIZE];
static uint16_t rx_head = 0;  
static uint16_t rx_tail = 0;  

static ComprehensivePose_t comprehensive_pose = {0};
float servo_current_angles[16] = {0.0f};

static int8_t motor_cmd_direction[4] = {0};
static float motor_cmd_speed[4] = {0.0f};
static uint8_t motor_cmd_type[4] = {0};
static osMutexId_t motor_cmd_mutex = NULL;

typedef struct {
    float start_angle;
    float target_angle;
    uint32_t start_time;
    uint32_t duration;
    uint8_t moving;
} ServoMotion_t;

static ServoMotion_t servo_motion[16] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// 内部解析函数声明为 static，避免外部链接冲突
static HAL_StatusTypeDef ParseServoControlMessage(uint8_t *buffer, uint16_t length);
static HAL_StatusTypeDef ParseBatchServoControlMessage(uint8_t *buffer, uint16_t length);
static HAL_StatusTypeDef ParseMotorControlMessage(uint8_t *buffer, uint16_t length);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t uart_rx_byte;

HAL_StatusTypeDef JetsonUart_StartReceiveIT(void)
{
    if(&huart5 == NULL) return HAL_ERROR;
    return HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5) {
        uint16_t next_head = (rx_head + 1) % RX_RING_BUFFER_SIZE;
        if(next_head != rx_tail) {
            rx_ring_buf[rx_head] = uart_rx_byte;
            rx_head = next_head;
        }
        HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
    }
}

void Jetson_HandleCommands(void)
{
    static uint8_t cmd_line[256];
    static uint16_t cmd_idx = 0;

    while(rx_tail != rx_head) {
        uint8_t b = rx_ring_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_RING_BUFFER_SIZE;

        if(b == '\n' || b == '\r') {
            if(cmd_idx > 0) {
                cmd_line[cmd_idx] = '\0';
                
                if(strncmp((char*)cmd_line, "BATCH_SERVO", 11) == 0) {
                    ParseBatchServoControlMessage(cmd_line, cmd_idx);
                } else if(strncmp((char*)cmd_line, "SERVO", 5) == 0) {
                    ParseServoControlMessage(cmd_line, cmd_idx);
                } else if(strncmp((char*)cmd_line, "MOTOR", 5) == 0) {
                    ParseMotorControlMessage(cmd_line, cmd_idx);
                }
                cmd_idx = 0; 
            }
        } else {
            if(cmd_idx < 255) {
                cmd_line[cmd_idx++] = b;
            } else { 
                cmd_idx = 0; 
            }
        }
    }
}

/* USER CODE END 0 */

static HAL_StatusTypeDef ParseServoControlMessage(uint8_t *buffer, uint16_t length)
{
    char *token = strtok((char*)buffer, ",");
    if(!token) return HAL_ERROR;

    token = strtok(NULL, ","); 
    if(!token) return HAL_ERROR;
    uint8_t id = atoi(token);

    token = strtok(NULL, ","); 
    if(!token) return HAL_ERROR;
    float angle = atof(token);

    token = strtok(NULL, ","); 
    if(!token) return HAL_ERROR;
    uint16_t duration = atoi(token);

    if(id >= 16) return HAL_ERROR;

    if(duration == 0) {
        servo_motion[id].moving = 0;
        servo_current_angles[id] = angle;
        extern I2C_HandleTypeDef hi2c2;
        PCA9685_SetAngleFloat(&hi2c2, id, angle);
    } else {
        servo_motion[id].start_angle = servo_current_angles[id];
        servo_motion[id].target_angle = angle;
        servo_motion[id].duration = duration;
        servo_motion[id].start_time = HAL_GetTick();
        servo_motion[id].moving = 1;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef ParseBatchServoControlMessage(uint8_t *buffer, uint16_t length)
{
    char *token = strtok((char*)buffer, ",");
    uint32_t now = HAL_GetTick();
    extern I2C_HandleTypeDef hi2c2;

    while((token = strtok(NULL, ",")) != NULL) {
        uint8_t id = atoi(token);
        token = strtok(NULL, ","); if(!token) break;
        float angle = atof(token);
        token = strtok(NULL, ","); if(!token) break;
        uint16_t duration = atoi(token);

        if(id < 16) {
            if(duration == 0) {
                servo_motion[id].moving = 0;
                servo_current_angles[id] = angle;
                PCA9685_SetAngleFloat(&hi2c2, id, angle);
            } else {
                servo_motion[id].start_angle = servo_current_angles[id];
                servo_motion[id].target_angle = angle;
                servo_motion[id].duration = duration;
                servo_motion[id].start_time = now;
                servo_motion[id].moving = 1;
            }
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef ParseMotorControlMessage(uint8_t *buffer, uint16_t length)
{
    char *token = strtok((char*)buffer, ",");
    token = strtok(NULL, ","); if(!token) return HAL_ERROR;
    uint8_t id = atoi(token);
    token = strtok(NULL, ","); if(!token) return HAL_ERROR;
    uint8_t cmd = atoi(token);
    token = strtok(NULL, ","); if(!token) return HAL_ERROR;
    float speed = atof(token);

    if(id < 4) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        motor_cmd_type[id] = cmd;
        motor_cmd_speed[id] = speed;
        if(cmd == 2) motor_cmd_direction[id] = -1;
        else if(cmd == 3) motor_cmd_direction[id] = 1;
        else motor_cmd_direction[id] = 0;
        __set_PRIMASK(primask);
    }
    return HAL_OK;
}

HAL_StatusTypeDef SendCurrentPose(void)
{
    char pose_msg[512];
    int len = sprintf(pose_msg, "POSE,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f\n",
            servo_current_angles[0], servo_current_angles[1],
            servo_current_angles[2], servo_current_angles[3],
            servo_current_angles[4], servo_current_angles[5],
            servo_current_angles[6], servo_current_angles[7],
            servo_current_angles[8], servo_current_angles[9],
            servo_current_angles[10], servo_current_angles[11],
            servo_current_angles[12], servo_current_angles[13],
            servo_current_angles[14], servo_current_angles[15],
            HAL_GetTick(),
            comprehensive_pose.imu_data.accel_x_g, comprehensive_pose.imu_data.accel_y_g, comprehensive_pose.imu_data.accel_z_g,
            comprehensive_pose.imu_data.gyro_x_dps, comprehensive_pose.imu_data.gyro_y_dps, comprehensive_pose.imu_data.gyro_z_dps,
            comprehensive_pose.imu_data.temperature_c,
            comprehensive_pose.motor_data.direction[0], comprehensive_pose.motor_data.direction[1],
            comprehensive_pose.motor_data.direction[2], comprehensive_pose.motor_data.direction[3],
            comprehensive_pose.motor_data.speed_rpm[0], comprehensive_pose.motor_data.speed_rpm[1],
            comprehensive_pose.motor_data.speed_rpm[2], comprehensive_pose.motor_data.speed_rpm[3]);

    return HAL_UART_Transmit(&huart5, (uint8_t*)pose_msg, len, 100);
}

void Servo_UpdateAll(void)
{
    uint32_t now = HAL_GetTick();
    extern I2C_HandleTypeDef hi2c2;
    extern void PCA9685_SetAngleFloat(I2C_HandleTypeDef *hi2c, uint8_t num, float angle);

    for (int id = 0; id < 16; id++) {
        if (!servo_motion[id].moving) continue;
        ServoMotion_t m = servo_motion[id];
        uint32_t elapsed = now - m.start_time;
        float cur;

        if (elapsed >= m.duration) {
            cur = m.target_angle;
            servo_motion[id].moving = 0;
        } else {
            float ratio = (float)elapsed / (float)m.duration;
            cur = m.start_angle + (m.target_angle - m.start_angle) * ratio;
        }
        servo_current_angles[id] = cur;
        comprehensive_pose.servo_pose.angles[id] = (uint16_t)(cur + 0.5f);
        PCA9685_SetAngleFloat(&hi2c2, id, cur);
    }
}

// 供其他模块更新数据
void UpdateIMUData(float ax, float ay, float az, float gx, float gy, float gz, float temp) {
    comprehensive_pose.imu_data.accel_x_g = ax;
    comprehensive_pose.imu_data.accel_y_g = ay;
    comprehensive_pose.imu_data.accel_z_g = az;
    comprehensive_pose.imu_data.gyro_x_dps = gx;
    comprehensive_pose.imu_data.gyro_y_dps = gy;
    comprehensive_pose.imu_data.gyro_z_dps = gz;
    comprehensive_pose.imu_data.temperature_c = temp;
}

void UpdateSingleMotorData(uint8_t id, int8_t dir, float rpm) {
    if(id < 4) {
        comprehensive_pose.motor_data.direction[id] = dir;
        comprehensive_pose.motor_data.speed_rpm[id] = rpm;
    }
}

uint8_t GetMotorCommandType(uint8_t id) { return (id < 4) ? motor_cmd_type[id] : 0; }
void GetMotorCommand(uint8_t id, int8_t *dir, float *speed) {
    if(id < 4) { *dir = motor_cmd_direction[id]; *speed = motor_cmd_speed[id]; }
}

HAL_StatusTypeDef JetsonUart_Init(void) {
    if (motor_cmd_mutex == NULL) {
        const osMutexAttr_t mutex_attr = { .name = "MotorCmdMutex" };
        motor_cmd_mutex = osMutexNew(&mutex_attr);
    }
    rx_head = rx_tail = 0;
    return HAL_OK;
}
// 占位函数兼容
void Jetson_CheckReceiveTimeout(void) {}
void Jetson_ClearStaleData(void) {}
uint8_t CheckPoseChanged(void) { return 1; }
