#include "pid.h"

/**
 * @brief  PID 初始化
 * @param  pid: 结构体指针
 * @param  kp/ki/kd: PID参数
 * @param  out_max: 输出限幅 (如PWM最大值)
 * @param  int_max: 积分限幅 (防止积分爆炸)
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float int_max) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->out_max = out_max;
    pid->integral_max = int_max;

    PID_Reset(pid); // 初始清零
}


/**
 * @brief  PID 核心计算函数
 * @param  pid: 结构体指针
 * @param  target: 期望值 (Target)
 * @param  current: 实际值 (Feedback)
 * @return 最终计算出的控制量 (Output)
 */
float PID_Calc(PID_TypeDef *pid, float target, float current) {
    pid->target = target;
    pid->current = current;

    // 1. 计算当前偏差
    pid->error = pid->target - pid->current;

    // 2. 积分直接累加
    pid->integral += pid->error;

    // 3. PID 公式计算 (先不算限幅)
    float p_term = pid->Kp * pid->error;
    float i_term = pid->Ki * pid->integral;
    float d_term = pid->Kd * (pid->error - pid->last_error);

    // 在这里对积分输出单独进行硬限幅 (防止积分爆炸)
    // 这样你的 MOTOR_SPEED_I_LIMIT 设成多少，I项就能结结实实贡献多少 PWM
    if (i_term > pid->integral_max)  i_term = pid->integral_max;
    if (i_term < -pid->integral_max) i_term = -pid->integral_max;

    pid->output = p_term + i_term + d_term;

    // 4. 更新上次误差，供下次微分计算使用
    pid->last_error = pid->error;

    // 5. 输出限幅
    if (pid->output > pid->out_max)  pid->output = pid->out_max;
    if (pid->output < -pid->out_max) pid->output = -pid->out_max;

    return pid->output;
}

/**
 * @brief  PID 数据清零 (通常在切换模式或电机重启时使用)
 */
void PID_Reset(PID_TypeDef *pid) {
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}