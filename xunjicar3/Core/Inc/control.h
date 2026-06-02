#ifndef __CONTROL_H
#define __CONTROL_H
#include "motor.h"


#define PULSES_PER_CM         73.0f //1cm的脉冲数
#define CHECK_WHITE_DISTANCE  (4.0f * PULSES_PER_CM)  // 盲走 4 厘米来确认是否是真全白
#define TARGET_STOP_DISTANCE  (25.0f * PULSES_PER_CM) // 确认终点后，再往前走 25 厘米停车
#define TARGET_CAMERA_STOP_DISTANCE     30.0f

//循迹传感器
#define MOTOR_TRACK_KP 0.8f // 比例系数
#define MOTOR_TRACK_KI 0.0f // 积分系数
#define MOTOR_TRACK_KD 1.2f // 微分系数
#define MOTOR_TRACK_I_LIMIT      1000.0f


// 1. X偏差转向环 (处理左右偏移，算出 turn_speed 差速项)
#define MOTOR_TURN_KP           5.0f
#define MOTOR_TURN_KI           0.0f
#define MOTOR_TURN_KD           1.2f
#define MOTOR_TURN_I_LIMIT      800.0f

// 2. 距离靠近环 (处理前后远近，算出 forward_speed 前进基础油门)
#define MOTOR_FORWARD_KP        -25.0f
#define MOTOR_FORWARD_KI        0.01f
#define MOTOR_FORWARD_KD        2.5f
#define MOTOR_FORWARD_I_LIMIT   200.0f


/**
 * @brief 小车全局顶级策略模式
 */
typedef enum {
    MODE_STANDBY       = 0,  // 待机模式/急停 (安全第一，所有输出归零)
    MODE_LINE_TRACKING = 1,  // 循迹模式 (主循环红外传感器接管)
    MODE_VISION_FOLLOW = 2,  // 视觉跟随模式 (OpenMV接管)
    MODE_BACKING       = 3,  // 找线模式
} CarMode_t;


//全局循迹控制结构体
typedef struct {
    PID_TypeDef track_pid;      // 循迹外环 PID 结构体
    float       track_error;    // 当前五路加权计算出来的偏差 (error)
    float       base_speed;     // 循迹时的全局基础直行速度 (脉冲/10ms 或 rps)

    float       standard_speed; // 随速映射的基准标准速度 (如你之前设定的常数)
    float       base_kp;        // 随速映射的基础 Kp
    float       base_kd;        // 随速映射的基础 Kd

    int8_t     last_dir;        //上一次方向
    int8_t     dir_locked;      //方向是否锁定
} Tracking_HandleTypeDef;

typedef enum {
    STOP_STATE_TRACKING = 0,   // 正常循迹，等待全黑
    STOP_STATE_ENTER_BLACK,    // 踩到全黑（可能是十字路口，也可能是终点）
    STOP_STATE_CHECK_WHITE,    // 全白校验状态：如果是十字路口，很快就会重新看到线
    STOP_STATE_MOVE_DISTANCE,  // 确认是终点，在长方形内定量前行
    STOP_STATE_FINISHED        // 到达指定距离，完全停车
} StopState_e;

extern PID_TypeDef turn_pid;
extern Tracking_HandleTypeDef tracker;
extern PID_TypeDef forward_pid;
extern CarMode_t car_mode;

extern uint8_t finish_camera_task;

// 功能函数接口
void Car_Set_Open_PWM(Motor_HandleTypeDef *hmotor,float pwm);
void Car_Set_Speed(Motor_HandleTypeDef *hmotor,float speed);
void Car_Set_Target_RPM(Motor_HandleTypeDef *hmotor,float rpm);
void Car_Move_Turns(Motor_HandleTypeDef *hmotor,float turns);
void Track_line(Tracking_HandleTypeDef *ptrack,Motor_HandleTypeDef *left_motor, Motor_HandleTypeDef *right_motor);
void Car_Strategy_Init();
void Motor_Tick_10ms(Motor_HandleTypeDef *hmotor);
void track_camera(float err_x, float distance);
#endif