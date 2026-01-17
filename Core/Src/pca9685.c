#include "pca9685.h"

/**
 * @brief  Set PCA9685 PWM frequency
 * @param  hi2c: I2C handle
 * @param  freq: PWM frequency
 * @retval None
 */
void PCA9685_SetFreq(I2C_HandleTypeDef *hi2c, float freq)
{
    uint8_t prescale, oldmode, newmode;
    double prescaleval;
    
    // Adjust frequency for PCA9685 internal oscillator
    freq *= 0.92;
    
    prescaleval = 25000000.0;  // PCA9685 internal clock is ~25MHz
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    
    prescale = (uint8_t)(prescaleval + 0.5f);

    // Read current mode
    uint8_t reg_data;
    HAL_I2C_Mem_Read(hi2c, PCA9685_ADDR, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
    oldmode = reg_data;

    // Go to sleep mode to change prescaler
    newmode = (oldmode & 0x7F) | 0x10; // sleep
    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &newmode, 1, 100);

    // Set the prescaler
    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, PCA9685_PRESCALE, I2C_MEMADD_SIZE_8BIT, &prescale, 1, 100);

    // Restore old mode
    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &oldmode, 1, 100);
    HAL_Delay(2);

    // Enable auto-increment
    uint8_t mode = oldmode | 0xa1;
    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &mode, 1, 100);
}

/**
 * @brief  Set PCA9685 PWM on/off time
 * @param  hi2c: I2C handle
 * @param  num: PWM channel (0-15)
 * @param  on: ON time (0-4096)
 * @param  off: OFF time (0-4096)
 * @retval None
 */
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t num, uint32_t on, uint32_t off)
{
    uint8_t data[4];
    
    data[0] = on & 0xFF;          // LEDx_ON_L
    data[1] = (on >> 8) & 0x0F;   // LEDx_ON_H
    data[2] = off & 0xFF;         // LEDx_OFF_L
    data[3] = (off >> 8) & 0x0F;  // LEDx_OFF_H
    
    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, PCA9685_LED0_ON_L + 4*num, I2C_MEMADD_SIZE_8BIT, data, 4, 100);
}

/**
 * @brief  Initialize PCA9685
 * @param  hi2c: I2C handle
 * @param  freq: PWM frequency
 * @retval None
 */
void PCA9685_Init(I2C_HandleTypeDef *hi2c, float freq)
{
    uint8_t mode1_data = 0x00;
    
    // Reset mode register
    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &mode1_data, 1, 100);
    
    // Set PWM frequency
    PCA9685_SetFreq(hi2c, freq);
    
    // Initialize all channels to 0
    for(uint8_t i = 0; i < 16; i++) {
        PCA9685_SetPWM(hi2c, i, 0, 0);
    }
    
    HAL_Delay(500);
}

/**
 * @brief  Set PCA9685 duty cycle for servo control
 * @param  hi2c: I2C handle
 * @param  num: PWM channel (0-15)
 * @param  duty: Duty cycle (0.0 - 100.0)
 * @retval None
 */
void PCA9685_SetDuty(I2C_HandleTypeDef *hi2c, uint8_t num, float duty)
{
    uint32_t off_time;
    
    // Limit duty cycle to 0-100%
    if(duty > 100.0f) duty = 100.0f;
    if(duty < 0.0f) duty = 0.0f;
    
    // Calculate off time (4096 corresponds to 100% duty cycle)
    off_time = (uint32_t)(duty * 40.96f);  // 4096 / 100 = 40.96
    
    PCA9685_SetPWM(hi2c, num, 0, off_time);
}

/**
 * @brief  Set PCA9685 angle for servo control
 * @param  hi2c: I2C handle
 * @param  num: PWM channel (0-15)
 * @param  angle: Angle (0-180 degrees)
 * @retval None
 */
void PCA9685_SetAngle(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t angle)
{
    uint32_t off_time;
    
    // Limit angle to 0-180
    if(angle > 180) angle = 180;
    
    // Calculate off time for servo angle (pulse width 0.5ms to 2.5ms for 50Hz)
    // 0.5ms = 110, 2.5ms = 525 (for 50Hz frequency)
    off_time = (uint32_t)(110 + angle * 2.27f);  // 2.27 = (525-110)/180
    
    PCA9685_SetPWM(hi2c, num, 0, off_time);
}

/**
 * @brief  Control servo motor with PCA9685
 * @param  hi2c: I2C handle
 * @param  num: Servo channel (0-15)
 * @param  start_angle: Start angle (0-180 degrees)
 * @param  end_angle: End angle (0-180 degrees)
 * @param  speed: Speed control (1-9, higher value means slower)
 * @retval None
 */
void PCA9685_ServoControl(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t start_angle, uint8_t end_angle, uint8_t speed)
{
    // For faster response, just set the final position
    uint32_t off_time = (uint32_t)(110 + end_angle * 2.27f);
    PCA9685_SetPWM(hi2c, num, 0, off_time);
}

/**
 * @brief  Control servo motor with PCA9685 with smooth movement
 * @param  hi2c: I2C handle
 * @param  num: Servo channel (0-15)
 * @param  start_angle: Start angle (0-180 degrees)
 * @param  end_angle: End angle (0-180 degrees)
 * @param  speed: Speed control (1-9, higher value means slower)
 * @retval None
 */
void PCA9685_ServoControlSmooth(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t start_angle, uint8_t end_angle, uint8_t speed)
{
    // 限制速度值在1-9范围内
    if(speed < 1) speed = 1;
    if(speed > 9) speed = 9;

    // 计算步长，速度值越高，步长越小（移动越慢）
    uint8_t step_size = (10 - speed); // 速度9对应步长1，速度1对应步长9
    if(step_size == 0) step_size = 1; // 确保至少移动1度

    uint32_t off_time;

    if(end_angle > start_angle)
    {
        // 从起始角度到结束角度，按步长递增
        for(uint8_t i = start_angle; i <= end_angle; i += step_size)
        {
            // 计算舵机角度对应的PWM值
            off_time = (uint32_t)(110 + i * 2.27f);
            PCA9685_SetPWM(hi2c, num, 0, off_time);

            // 添加小延时以实现平滑移动
            HAL_Delay(10);

            // 如果下一次迭代会超过目标角度，则直接设置到目标角度
            if(i + step_size > end_angle) {
                off_time = (uint32_t)(110 + end_angle * 2.27f);
                PCA9685_SetPWM(hi2c, num, 0, off_time);
                break;
            }
        }
    }
    else if(end_angle < start_angle)
    {
        // 从起始角度到结束角度，按步长递减
        for(uint8_t i = start_angle; i >= end_angle; i -= step_size)
        {
            // 计算舵机角度对应的PWM值
            off_time = (uint32_t)(110 + i * 2.27f);
            PCA9685_SetPWM(hi2c, num, 0, off_time);

            // 添加小延时以实现平滑移动
            HAL_Delay(10);

            // 如果下一次迭代会低于目标角度，则直接设置到目标角度
            if(i < step_size || (i - step_size) < end_angle) {
                off_time = (uint32_t)(110 + end_angle * 2.27f);
                PCA9685_SetPWM(hi2c, num, 0, off_time);
                break;
            }
        }
    }
    else
    {
        // 如果起始角度和结束角度相同，直接设置到该角度
        off_time = (uint32_t)(110 + start_angle * 2.27f);
        PCA9685_SetPWM(hi2c, num, 0, off_time);
    }
}