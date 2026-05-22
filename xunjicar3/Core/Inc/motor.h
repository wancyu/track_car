#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f4xx_hal.h"





// 基础物理参数定义：JGB37-520 电机参数 (12V 330RPM版本)
#define MOTOR_RATED_RPM          330.0f  // 官方额定转速：330 转/分钟 (RPM)
#define ENCODER_REDUCTION_RATIO  34.0f   // 减速比：1:34
#define ENCODER_PPR              11.0f   // 编码器线数：磁环单相 11 个脉冲/ 轮子旋转一整圈，单片机 4 倍频捕获到的总脉冲数：11 * 4 * 34 = 1496.0f


// 轮子转一圈总脉冲数：11 * 4 * 34 = 1496.0f
#define ENCODER_TOTAL_PPR        (ENCODER_PPR * 4.0f * ENCODER_REDUCTION_RATIO)
// 电机物理极限最大转速 (转/秒)：330.0f / 60.0f = 5.5f r/s
#define MOTOR_MAX_RPS            (MOTOR_RATED_RPM / 60.0f)
// 硬件 速度 最大限制
#define SPEED_OUTPUT_MAX         MOTOR_MAX_RPS * (ENCODER_TOTAL_PPR / 100.0f)
// 硬件 PWM 最大限制
#define MOTOR_PWM_MAX            1000    // PWM 最大输出值 (对应定时器 ARR)

// --- 速度环 PID 参数 (Speed Loop) ---
#define MOTOR_SPEED_KP           0.01f   // 比例系数
#define MOTOR_SPEED_KI           0.0005f   // 积分系数
#define MOTOR_SPEED_KD           0.00f   // 微分系数
#define MOTOR_SPEED_I_LIMIT      40.0f  // 积分限幅 (防止积分饱和)

// --- 位置环 PID 参数 (Position Loop - 轨迹规划辅助) ---
#define MOTOR_POS_KP             0.50f
#define MOTOR_POS_KI             0.00f
#define MOTOR_POS_KD             0.00f


// 1. 电机模式枚举 (比用 0,1,2 更直观)
typedef enum {
    MOTOR_MODE_OPEN_LOOP = 0, // 开环
    MOTOR_MODE_SPEED,         // 单速度闭环
    MOTOR_MODE_POSITION       // 串级位置闭环
} MotorMode_e;


// 2. 电机结构体
typedef struct {
    // --- 硬件映射层 ---
    GPIO_TypeDef*       IN1_Port;
    uint16_t            IN1_Pin;
    GPIO_TypeDef*       IN2_Port;
    uint16_t            IN2_Pin;
    TIM_HandleTypeDef*  PWM_Tim;
    uint32_t            PWM_Channel;
    TIM_HandleTypeDef*  ENC_Tim;

    // --- 物理状态层 ---
    float               pwm_target;  // 开环模式下的目标占空比
    uint16_t            pwm_duty;    // 当前实际输出的占空比 (0-1000)

    float               speed_target; //目标速度
    float               speed;        // 当前速度 (脉冲/10ms)
    uint32_t            last_cnt;     // 上一次的速度

    float               rps_target;       // 目标转速
    float               rps;             // 当前转速

    int64_t             pos_target;  // 目标位置
    int64_t             pos;         // 绝对脉冲位置
    float               turns_target;  // 目标圈数
    float               turns;         // 当前圈数

    // --- 算法控制层 ---
    // PID_TypeDef         speed_pid;
    // PID_TypeDef         pos_pid;

    // --- 状态选择 ---
    MotorMode_e         ctrl_mode;
} Motor_HandleTypeDef;



extern Motor_HandleTypeDef MotorA;


/**
 * @brief 电机初始化函数 (带参数，一套代码初始化所有电机)
 */
void Motor_Init(Motor_HandleTypeDef *hmotor,
                GPIO_TypeDef* port1, uint16_t pin1,
                GPIO_TypeDef* port2, uint16_t pin2,
                TIM_HandleTypeDef* pwm_tim, uint32_t channel,
                TIM_HandleTypeDef* enc_tim);

void Motor_Set_PWM(Motor_HandleTypeDef *hmotor, int32_t pwm_val);
void Motor_Tick_10ms(Motor_HandleTypeDef *hmotor);
#endif