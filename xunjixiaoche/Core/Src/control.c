#include "control.h"

// ================== 1. 外部硬件引用 ==================
// 声明我们在 CubeMX 里配好的定时器，用于读取编码器
extern TIM_HandleTypeDef htim4;

// ================== 2. 实例化控制对象 ==================
PID_TypeDef Speed_PID;   // 内环：速度环
PID_TypeDef Vision_PID;  // 外环：视觉位置环

// 视觉数据缓存
static float current_vision_error = 0.0f;
static uint8_t vision_update_flag = 0;   // 0:无新数据 1:有新数据

// ================== 3. 内部私有函数：PID 计算 ==================
// static 关键字意味着这个函数只能在 control.c 内部使用，外部看不见，防止被乱调
static float PID_Calc(PID_TypeDef *pid, float target, float actual)
{
    pid->error = target - actual;

    // 积分限幅
    pid->integral += pid->error;
    if (pid->integral > pid->max_integral)       pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    // 计算输出
    pid->output = (pid->kp * pid->error) +
                  (pid->ki * pid->integral) +
                  (pid->kd * (pid->error - pid->last_error));

    pid->last_error = pid->error;

    // 输出限幅
    if (pid->output > pid->max_output)       pid->output = pid->max_output;
    else if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    return pid->output;
}

// 内部私有函数：PID 参数初始化
static void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_i, float max_out)
{
    pid->kp = p; pid->ki = i; pid->kd = d;
    pid->max_integral = max_i; pid->max_output = max_out;
    pid->error = 0; pid->last_error = 0; pid->integral = 0; pid->output = 0;
}


// ================== 4. 对外提供的接口实现 ==================

// 接口1：初始化
void Control_Init(void)
{
    // 参数根据你的车体实际情况调整
    // 速度环: Kp, Ki, Kd, 积分限幅, 对应PWM/电流的输出限幅
    PID_Init(&Speed_PID,  1.2f, 0.5f, 0.1f, 1000.0f, 5000.0f);

    // 视觉环: 输出限幅(1500)即为允许小车跑出的【最大目标速度】
    PID_Init(&Vision_PID, 0.5f, 0.0f, 0.1f, 300.0f,  1500.0f);
}

// 接口2：更新视觉数据
void Control_Set_Vision_Error(float x_error)
{
    current_vision_error = x_error;
    vision_update_flag = 1; // 立起 Flag，告诉 10ms 中断有新数据了
}

// 接口3：核心 10ms 控制流
void Control_10ms_Routine(void)
{
    float target_speed = 0.0f;

    // 【第 1 步】：获取底层当前实际速度 (读编码器并清零)
    short current_speed = (short)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // 【第 2 步】：外环计算 (视觉误差 -> 目标速度)
    if (vision_update_flag == 1)
    {
        // 视觉死区消抖处理 (如果是5像素内的极小晃动，视为0误差)
        if (current_vision_error > -5.0f && current_vision_error < 5.0f)
        {
            current_vision_error = 0.0f;
        }

        // 视觉环的目标永远是 0 (即目标在画面正中央，误差为0)
        target_speed = PID_Calc(&Vision_PID, 0.0f, current_vision_error);

        vision_update_flag = 0; // 用完即销毁标志位
    }
    else
    {
        // 安全机制：如果 K210 卡死没发数据，让小车减速停车，防止失控撞墙
        // target_speed = 0.0f;

        // 如果想让它在两帧视觉数据之间保持上一帧的速度平滑运行，可以把上面那句注释掉
    }

    // 【第 3 步】：内环计算 (目标速度 -> 驱动指令)
    float motor_command = PID_Calc(&Speed_PID, target_speed, current_speed);

    // 【第 4 步】：下发到底层电机驱动板
    // Send_Command_To_DGM(motor_command);
}