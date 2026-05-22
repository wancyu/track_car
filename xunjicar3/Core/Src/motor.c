#include "motor.h"

Motor_HandleTypeDef MotorA;




/**
 * @brief  电机对象初始化函数
 * @param  hmotor: 电机结构体指针
 * @param  port1: 电机方向引脚类型
 * @param  pin1: 电机方向引脚1 (AIN1)
 * @param  port2:
 * @param  pin2: 电机方向引脚2 (AIN2)
 * @param  pwm_tim: PWM输出定时器及通道
 * @param  channel: pwm输出
 * @param  enc_tim: 编码器读取定时器
 * @note   该函数完成了硬件绑定、外设启动、PID初始化及规划器初始化
 */
void Motor_Init(Motor_HandleTypeDef *hmotor,
                GPIO_TypeDef* port1, const uint16_t pin1,
                GPIO_TypeDef* port2, const uint16_t pin2,
                TIM_HandleTypeDef* pwm_tim, const uint32_t channel,
                TIM_HandleTypeDef* enc_tim)
{
    // 硬件引脚与外设句柄绑定
    hmotor->IN1_Port = port1;
    hmotor->IN1_Pin  = pin1;
    hmotor->IN2_Port = port2;
    hmotor->IN2_Pin  = pin2;
    hmotor->PWM_Tim  = pwm_tim;
    hmotor->PWM_Channel = channel;
    hmotor->ENC_Tim  = enc_tim;

    // 强制让方向引脚全部拉低，让驱动芯片（TB6612）进入死区刹车状态，严防起步抽搐
    HAL_GPIO_WritePin(hmotor->IN1_Port, hmotor->IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hmotor->IN2_Port, hmotor->IN2_Pin, GPIO_PIN_RESET);

    // PID 参数初始化
    // 速度环初始化
    //PID_Init(&hmotor->speed_pid, MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD, MOTOR_PWM_MAX, MOTOR_SPEED_I_LIMIT);
    // 位置环初始化
    //PID_Init(&hmotor->pos_pid, MOTOR_POS_KP, MOTOR_POS_KI, MOTOR_POS_KD, SPEED_OUTPUT_MAX, 0.0f);

    // 物理状态初始化
    hmotor->pwm_target   = 0.0f;
    hmotor->speed_target = 0.0f;
    hmotor->rps_target   = 0.0f;
    hmotor->turns_target = 0.0f;
    hmotor->pos          = 0;
    hmotor->pos_target   = 0;
    hmotor->turns        = 0.0f;
    hmotor->speed        = 0.0f;
    hmotor->rps          = 0.0f;
    hmotor->pwm_duty     = 0;
    hmotor->ctrl_mode    = MOTOR_MODE_OPEN_LOOP; // 默认为开环模式

    // 硬件编码器定时器计数
    HAL_TIM_Encoder_Start(hmotor->ENC_Tim, TIM_CHANNEL_ALL);

    // 用当前最真实的物理值覆盖 last_cnt
    hmotor->last_cnt = __HAL_TIM_GET_COUNTER(enc_tim);

    // 解开底层 PWM 的硬输出
    __HAL_TIM_SET_COMPARE(hmotor->PWM_Tim, hmotor->PWM_Channel, 0); // 确保初始占空比是0
    HAL_TIM_PWM_Start(hmotor->PWM_Tim, hmotor->PWM_Channel);
}


/**
 * @brief  2. 底层驱动与限幅 (内部私有函数)
 * @param  pwm_val: PID计算出来的有符号输出
 */
void Motor_Set_PWM(Motor_HandleTypeDef *hmotor, int32_t pwm_val)
{
    // --- 这里的限幅非常重要 ---
    // 假设你的定时器重装载值(ARR)是 1000，那么PWM绝对不能超过这个数
    if(pwm_val > MOTOR_PWM_MAX)  pwm_val = MOTOR_PWM_MAX;
    if(pwm_val < -MOTOR_PWM_MAX) pwm_val = -MOTOR_PWM_MAX;

    // 根据正负决定 TB6612 的方向
    if (pwm_val > 0) {
        HAL_GPIO_WritePin(hmotor->IN1_Port, hmotor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(hmotor->IN2_Port, hmotor->IN2_Pin, GPIO_PIN_RESET);
        hmotor->pwm_duty = pwm_val;
    } else if (pwm_val < 0) {
        HAL_GPIO_WritePin(hmotor->IN1_Port, hmotor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hmotor->IN2_Port, hmotor->IN2_Pin, GPIO_PIN_SET);
        hmotor->pwm_duty = -pwm_val; // 转换为正数给定时器
    } else {
        HAL_GPIO_WritePin(hmotor->IN1_Port, hmotor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hmotor->IN2_Port, hmotor->IN2_Pin, GPIO_PIN_RESET);
        hmotor->pwm_duty = 0;
    }

    // 下发占空比到硬件
    __HAL_TIM_SET_COMPARE(hmotor->PWM_Tim, hmotor->PWM_Channel, hmotor->pwm_duty);
}



void Motor_Tick_10ms(Motor_HandleTypeDef *hmotor) {
    // --- A. 感知层：读编码器 (F411 兼容 16/32 位版本) ---
    uint32_t current_cnt = __HAL_TIM_GET_COUNTER(hmotor->ENC_Tim);
    int32_t delta = 0;

    if (hmotor->ENC_Tim->Instance == TIM2 || hmotor->ENC_Tim->Instance == TIM5)
    {
        delta = (int32_t)(current_cnt - hmotor->last_cnt);
    }
    else
    {
        delta = (int16_t)((uint16_t)current_cnt - (uint16_t)hmotor->last_cnt);
    }

    hmotor->speed = (float)delta; // 速度获取
    hmotor->last_cnt = current_cnt;
}