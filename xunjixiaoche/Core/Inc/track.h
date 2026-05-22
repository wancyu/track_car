#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f4xx_hal.h"


// 1. 定义小车运行状态枚举
typedef enum {
    MODE_TRACK = 0,     // 常规循迹模式
    MODE_PARK_DRIVE,    // 进库盲走模式
    MODE_STOP,          // 彻底停车模式
    MODE_CAMERA,        // 摄像头视觉对齐模式
    MODE_WAIT,          // 停车等待3秒模式
    MODE_REVERSE        // 倒车模式
} CarMode;

// 2. 声明全局变量（供跨文件使用）
extern CarMode car_mode;
extern uint32_t action_timer;
// 摄像头相关变量
extern float cam_err;     // 摄像头传来的中心点误差
extern float cam_dist;    // 摄像头传来的距离 (cm)

// 3. 声明新函数
void Camera_PD(void);
void Tracking_PD(void);

#endif