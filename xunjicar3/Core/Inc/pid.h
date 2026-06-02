#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h" // 只要能用到 float 类型即可

/* PID 结构体定义 */
typedef struct {
    // 1. PID 三项参数
    float Kp;
    float Ki;
    float Kd;

    // 2. 目标值与反馈值
    float target;
    float current;

    // 3. 过程变量
    float error;
    float last_error;
    float integral;

    // 4. 限幅保护
    float out_max;      // 输出限幅
    float integral_max; // 积分限幅 (防止积分饱和)

    // 5. 计算结果
    float output;
} PID_TypeDef;



/* 函数声明 */
void  PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float int_max);
float PID_Calc(PID_TypeDef *pid, float target, float current);
void  PID_Reset(PID_TypeDef *pid);

#endif