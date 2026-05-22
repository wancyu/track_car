#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"  // 必须包含，为了使用 uint8_t 和 HAL 库外设

// ================== 1. PID 结构体定义 ==================
// 把 PID 的“收纳盒”图纸放在这里，如果 main 里想看数据也能访问
typedef struct {
    float kp, ki, kd;
    float error, last_error, integral;
    float max_integral, max_output;
    float output;
} PID_TypeDef;

// ================== 2. 对外接口声明 ==================

// 接口1：初始化控制系统 (在 main 函数的 while(1) 前调用)
void Control_Init(void);

// 接口2：喂给控制系统最新的视觉误差 (在 UART DMA 接收完成回调中调用)
// 比如：传入目标在画面中心的偏差像素值 x_error
void Control_Set_Vision_Error(float x_error);

// 接口3：10ms 控制节拍器 (在 TIM3 的 10ms 溢出中断中调用)
void Control_10ms_Routine(void);

#endif