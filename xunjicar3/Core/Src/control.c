#include "control.h"
#include <stdlib.h>



StopState_e Current_Stop_State; //循迹状态枚举
Tracking_HandleTypeDef tracker; //循迹控制结构体
PID_TypeDef turn_pid;
PID_TypeDef forward_pid;

CarMode_t car_mode; //小车运行模式枚举
int64_t start_check_pos = 0; // 记录开始校验全白时的绝对脉冲
int64_t start_stop_pos = 0;  // 记录开始计距停车时的绝对脉冲
uint8_t finish_camera_task = 0;  //完成视觉识别任务标志
uint32_t alarm_tick = 0; //上一次鸣笛时间

int s1,s2,s3,s4,s5;
int sum, cnt;
/**
 * @brief  上层策略与外环 PID 初始化
 */
void Car_Strategy_Init(void)
{
    // --- 1. 默认状态设置 ---
    car_mode = MODE_LINE_TRACKING;
    Current_Stop_State = 0;
    tracker.dir_locked = 0;
    // --- 2. 循迹环 PID 初始化 ---
    PID_Init(&tracker.track_pid,
             MOTOR_TRACK_KP, MOTOR_TRACK_KI, MOTOR_TRACK_KD,
             SPEED_OUTPUT_MAX, MOTOR_TRACK_I_LIMIT);

    // 视觉转向 PID 初始化 (处理 X 偏差)
    PID_Init(&turn_pid,
             MOTOR_TURN_KP, MOTOR_TURN_KI, MOTOR_TURN_KD,
             SPEED_OUTPUT_MAX, MOTOR_TURN_I_LIMIT);

    // 视觉定距 PID 初始化 (处理 Distance 偏差)
    PID_Init(&forward_pid,
             MOTOR_FORWARD_KP, MOTOR_FORWARD_KI, MOTOR_FORWARD_KD,
             SPEED_OUTPUT_MAX, MOTOR_FORWARD_I_LIMIT);
}

void alarm()
{
    Car_Set_Speed(&MotorA, 0.0f);
    Car_Set_Speed(&MotorB, 0.0f);
    // HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
    if (HAL_GetTick() - alarm_tick >= 3000)
    {
        Car_Set_Speed(&MotorA, 0.0f);
        Car_Set_Speed(&MotorB, 0.0f);
        // HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
         car_mode = MODE_BACKING;
    }
}


void track_camera(float err_x, float distance)
{
    // 1. 核心闭环计算（保持我们调好的 -25.0f 和 5.0f 的平稳外环参数）
    float turn_speed = PID_Calc(&turn_pid, 0.0f, err_x);
    float forward_speed = PID_Calc(&forward_pid, (float)TARGET_CAMERA_STOP_DISTANCE, distance);

    // 精密定距死区管理
    float current_error = distance - TARGET_CAMERA_STOP_DISTANCE;
    if (current_error > -1.5f && current_error < 1.5f)
    {
        forward_speed = 0.0f;
        PID_Reset(&forward_pid);
        finish_camera_task = 1;
        car_mode = MODE_STANDBY;             // 切换到鸣笛状态
    }

    // 2. 差速混合
    float raw_target_L = forward_speed - turn_speed;
    float raw_target_R = forward_speed + turn_speed;

    // 刚性外环最大速度限制
    // 既然底层极速是 3000，我们在视觉模式下把喂给底层的目标速度死死卡在 1200.0f
    // 既保证了极高的追击速度，又绝对不会让外环把内环打到深度饱和！
    float max_pos_out = 1200.0f;
    if (raw_target_L > max_pos_out)  raw_target_L = max_pos_out;
    if (raw_target_L < -max_pos_out) raw_target_L = -max_pos_out;
    if (raw_target_R > max_pos_out)  raw_target_R = max_pos_out;
    if (raw_target_R < -max_pos_out) raw_target_R = -max_pos_out;

    // 4.工业级一阶低通滤波器
    static float filtered_L = 0.0f;
    static float filtered_R = 0.0f;

    // 滤波系数 0.20f。把 1300 到 1400 的刚性阶跃，熨平成极其丝滑的平滑斜坡
    filtered_L = (filtered_L * 0.80f) + (raw_target_L * 0.20f);
    filtered_R = (filtered_R * 0.80f) + (raw_target_R * 0.20f);

    // 5. 最终顺畅输出给底层速度环
    Car_Set_Speed(&MotorA, filtered_L);
    Car_Set_Speed(&MotorB, filtered_R);
}

/**
 * @brief 解耦全局传感器扫描与方向记忆，永远在Track_line()前运行
 */
void Sensor_Global_Scan(Tracking_HandleTypeDef *ptrack)
{
    s1 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    s2 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
    s3 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
    s4 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
    s5 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

    sum = 0, cnt = 0;
    if (s1) { sum += -2; cnt++; }
    if (s2) { sum += -1; cnt++; }
    if (s3) { sum +=  0; cnt++; }
    if (s4) { sum +=  1; cnt++; }
    if (s5) { sum +=  2; cnt++; }

    // 视觉模式，白线，方向锁定
    if (car_mode == MODE_VISION_FOLLOW && cnt == 0)
    {
        ptrack->dir_locked = 1;
    }
    // 找线模式，方向解锁
    if (car_mode == MODE_BACKING)
    {
        ptrack->dir_locked = 0;
    }
    //方向未锁定，有线
    if (cnt != 0 && ptrack->dir_locked == 0)
    {

        if (sum < 0)       ptrack->last_dir = -1; // 线在左边
        else if (sum > 0)  ptrack->last_dir = 1;  // 线在右边
        else               ptrack->last_dir = 0;  // 线在正中
    }
    else
    {
        cnt = 0; // 地上没线了
    }
}


/**
 * @brief 速度映射的位置环算法
 */
void Track_line(Tracking_HandleTypeDef *ptrack,Motor_HandleTypeDef *left_motor, Motor_HandleTypeDef *right_motor)
{
    //sum，cnt，s1-s5 数据已写入本文件定义的变量中
    ptrack->dir_locked = 0;
    switch (Current_Stop_State)
    {
    case STOP_STATE_TRACKING:
        // 5路全黑，触发全黑状态，无法区分十字路口还是终点
        if (s1 && s2 && s3 && s4 && s5)
        {
            Current_Stop_State = STOP_STATE_ENTER_BLACK;
        }
        break;

    case STOP_STATE_ENTER_BLACK:
        // 等待车头探出全黑区域。一旦全白(cnt == 0)，说明冲出了横线，进入全白校验期
        if (cnt == 0)
        {
            start_check_pos = left_motor->pos; // 记录当前编码器位置
            Current_Stop_State = STOP_STATE_CHECK_WHITE;
        }
        break;

    case STOP_STATE_CHECK_WHITE:
        // 在校验期内,盲走一定距离，如果又检测到了黑线 (cnt != 0)，说明刚才那是个十字路口！
        if (cnt != 0)
        {
            Current_Stop_State = STOP_STATE_TRACKING; // 判定误报，退回正常循迹模式
        }
        // 如果稳稳当当地盲走了 4 厘米，期间没有任何传感器碰到黑线，说明这绝对是终点的全白广场
        else if (llabs(left_motor->pos - start_check_pos) >= (int64_t)CHECK_WHITE_DISTANCE)
        {
            start_stop_pos = left_motor->pos; // 开始计算最后的停车距离
            Current_Stop_State = STOP_STATE_MOVE_DISTANCE;
        }
        break;

    case STOP_STATE_MOVE_DISTANCE:
        // 确认进入终点框，直线匀速走完最后一段距离
        if (llabs(left_motor->pos - start_stop_pos) >= (int64_t)TARGET_STOP_DISTANCE)
        {
            Current_Stop_State = STOP_STATE_FINISHED;
        }
        break;

    case STOP_STATE_FINISHED:
        // 彻底刹车
        break;
    }

    // --- 2. 行为分发（动作路由） ---
    if (Current_Stop_State == STOP_STATE_TRACKING || Current_Stop_State == STOP_STATE_ENTER_BLACK)
    {
        // 正常循迹
        if (cnt != 0) { ptrack->track_error = (float)sum / (float)cnt; }
        const float speed_ratio = ptrack->base_speed / ptrack->standard_speed;
        ptrack->track_pid.Kp = ptrack->base_kp * speed_ratio;
        ptrack->track_pid.Kd = ptrack->base_kd * speed_ratio;

        float bias_output = PID_Calc(&ptrack->track_pid, 0.0f, ptrack->track_error);
        left_motor->speed_target  = ptrack->base_speed + bias_output;
        right_motor->speed_target = ptrack->base_speed - bias_output;

        Car_Set_Speed(left_motor, ptrack->base_speed + bias_output);
        Car_Set_Speed(right_motor, ptrack->base_speed - bias_output);
    }
    else if (Current_Stop_State == STOP_STATE_CHECK_WHITE || Current_Stop_State == STOP_STATE_MOVE_DISTANCE)
    {
        // 在“全白校验期”和“确认终点后前行期”，地上都是全白，
        // 此时直接闭环走直线（单速度模式），不受任何循迹偏差干扰
        Car_Set_Speed(left_motor, ptrack->base_speed);
        Car_Set_Speed(right_motor, ptrack->base_speed);


    }
    else if (Current_Stop_State == STOP_STATE_FINISHED)
    {
        if (left_motor->ctrl_mode != MOTOR_MODE_POSITION)
        {
            PID_Reset(&left_motor->pos_pid);
            PID_Reset(&left_motor->speed_pid);
            PID_Reset(&right_motor->pos_pid);
            PID_Reset(&right_motor->speed_pid);

            Car_Move_Turns(left_motor, 0.0f);
            Car_Move_Turns(right_motor, 0.0f);
        }
    }
}


/**
 * @brief 找线模式（依靠脱轨前冻结的记忆切回黑线，融入低通滤波与刚性限速）
 * @note  对应应用层 control 中的返回找线功能，高频 10ms 周期性调用
 */
void Back_line(Tracking_HandleTypeDef *ptrack)
{

    float raw_target_L = 0.0f;
    float raw_target_R = 0.0f;

    // 用正常循迹速度的 65% ~ 70% 即可。慢一点，传感器摸到线时不容易因为惯性冲过头
    float recovery_base = ptrack->base_speed * 0.7f;

    // 根据 Sensor_Global_Scan 里死死冻结的 last_dir 给出非对称开环差速
    if (ptrack->last_dir == -1)
    {
        // 记忆中线在左边（车在右）：向左前方弧线内切（左轮慢，右轮快）
        raw_target_L = recovery_base * 0.3f;
        raw_target_R = recovery_base * 1.4f;
    }
    else if (ptrack->last_dir == 1)
    {
        // 记忆中线在右边（车在左）：向右前方弧线内切（左轮快，右轮慢）
        raw_target_L = recovery_base * 1.4f;
        raw_target_R = recovery_base * 0.3f;
    }
    else
    {
        // 如果最后记忆在线正中（如直行脱轨盲区），则保守直行试探
        raw_target_L = recovery_base;
        raw_target_R = recovery_base;
    }

    // 刚性外环最大速度限制
    // 死死卡在 1200.0f 保护内环不进入深度饱和
    float max_pos_out = 1200.0f;
    if (raw_target_L > max_pos_out)  raw_target_L = max_pos_out;
    if (raw_target_L < -max_pos_out) raw_target_L = -max_pos_out;
    if (raw_target_R > max_pos_out)  raw_target_R = max_pos_out;
    if (raw_target_R < -max_pos_out) raw_target_R = -max_pos_out;

    // 一阶低通滤波器
    // 复用平稳的 0.20f 熨平系数。
    // 作用：让小车从鸣笛静止状态到大差速弧线转弯的过程，变成极其丝滑的平滑斜坡，杜绝底盘猛烈抖动
    static float filtered_L = 0.0f;
    static float filtered_R = 0.0f;

    filtered_L = (filtered_L * 0.80f) + (raw_target_L * 0.20f);
    filtered_R = (filtered_R * 0.80f) + (raw_target_R * 0.20f);

    // 5顺畅输出给底层速度环
    Car_Set_Speed(&MotorA, filtered_L);
    Car_Set_Speed(&MotorB, filtered_R);

    // 自动接轨退出机制（闭环回归）
    // 只要后台一直运行的全局扫描发现任意红外探头重新压回黑线（cnt != 0）
    if (cnt != 0)
    {
        // 瞬间切回正常循迹模式，控制权交还给 Track_line
        car_mode = MODE_LINE_TRACKING;

        // 主动解开方向记忆锁，为下一次接近物体做准备
        ptrack->dir_locked = 0;

        // 关键一步：手动清空当前滤波器的静态局部变量，防止下次进入该模式时产生历史数据残留
        filtered_L = 0.0f;
        filtered_R = 0.0f;
    }
}

/**
 * @brief  固定PWM输出 (开环测试)
 * @param hmotor
 * @param  pwm: 范围 -1000 到 1000
 */
void Car_Set_Open_PWM(Motor_HandleTypeDef *hmotor, const float pwm) {
    hmotor->ctrl_mode = MOTOR_MODE_OPEN_LOOP;
    hmotor->pwm_target = pwm;
}


/**
 * @brief  固定速度输出 (速度环测试)
 * @param hmotor
 * @param  speed:
 */
void Car_Set_Speed(Motor_HandleTypeDef *hmotor, const float speed) {
    hmotor->ctrl_mode = MOTOR_MODE_SPEED;
    hmotor->speed_target = speed;
    hmotor->speed_pid.target= hmotor->speed_target;
}


/**
 * @brief  固定转速 (速度环)
 * @param hmotor
 * @param  rps: 目标转速 (单位: 转/秒)-5.5~5.5
 */
void Car_Set_Target_RPM(Motor_HandleTypeDef *hmotor, const float rps) {
    hmotor->ctrl_mode = MOTOR_MODE_SPEED;
    hmotor->rps_target = rps;
    // 转化公式：目标速度(脉冲/10ms) = 目标转速(r/s) * 一圈总脉冲 / 100
    hmotor->speed_target = (hmotor->rps_target * ENCODER_TOTAL_PPR) / 100.0f;
    hmotor->speed_pid.target = hmotor->speed_target;
}


/**
 * @brief  功能3：转指定圈数 (位置环)
 * @param  hmotor
 * @param  turns: 增加圈数 (单位: 圈)
 * @note   是在当前位置的基础上增加圈数
 */
void Car_Move_Turns(Motor_HandleTypeDef *hmotor, const float turns) {
    hmotor->ctrl_mode = MOTOR_MODE_POSITION;
    // 转化公式：目标圈数 = 当前位置(脉冲)/一圈总脉冲 + 增加圈数
    hmotor->turns_target = (float)hmotor->pos/ENCODER_TOTAL_PPR +turns;
    // 转化公式：目标位置(脉冲) = 目标圈数 * 一圈总脉冲
    hmotor->pos_target = (int64_t)(hmotor->turns_target * ENCODER_TOTAL_PPR);
}


/**
 * @brief  更新电机编码器数据及速度计算
 * @note
 */
static void Motor_Update_Encoder(Motor_HandleTypeDef *hmotor) {
    uint32_t current_cnt = __HAL_TIM_GET_COUNTER(hmotor->ENC_Tim);
    int32_t delta = 0;

    // 区分 32位 (TIM2/TIM5) 与 16位 定时器的溢出处理逻辑
    if (hmotor->ENC_Tim->Instance == TIM2 || hmotor->ENC_Tim->Instance == TIM5) {
        delta = (int32_t)(current_cnt - hmotor->last_cnt);
    } else {
        delta = (int16_t)((uint16_t)current_cnt - (uint16_t)hmotor->last_cnt);
    }

    // 状态更新
    hmotor->pos += delta;
    hmotor->speed = (float)delta;
    hmotor->rps = (hmotor->speed * 100.0f) / ENCODER_TOTAL_PPR;
    hmotor->turns = (float)hmotor->pos / ENCODER_TOTAL_PPR;
    hmotor->last_cnt = current_cnt;
}


/**
 * @brief  决策层：电机控制模式路由 与 PID 计算
 */
static void Motor_Calculate_Control(Motor_HandleTypeDef *hmotor) {
    float out_pwm = 0;

    switch (hmotor->ctrl_mode) {
    case MOTOR_MODE_POSITION:
        {
            // 用整型相减防 float 精度丢失，随后将误差送入PID
            const float pos_error = (float)(hmotor->pos_target - hmotor->pos);

            // 外环（位置环）：输入位置误差，输出目标速度
            const float target_speed = PID_Calc(&hmotor->pos_pid, pos_error, 0.0f);
            hmotor->speed_pid.target = target_speed;

            // 内环（速度环）：输入目标速度和当前速度，输出 PWM
            out_pwm = PID_Calc(&hmotor->speed_pid, hmotor->speed_pid.target, hmotor->speed);
            break;
        }

    case MOTOR_MODE_SPEED:
        {
            out_pwm = PID_Calc(&hmotor->speed_pid, hmotor->speed_pid.target, hmotor->speed);
            break;
        }

    case MOTOR_MODE_OPEN_LOOP: // --- 3. 开环模式 (PWM 直通) ---
    default:
        {
            out_pwm = hmotor->pwm_target;
            break;
        }
    }
    Motor_Set_PWM(hmotor, (int32_t)out_pwm);
}


/**
 * @brief  3. 10ms 周期调用函数
 * @note   放置在定时器中断中，负责感知、决策、执行全流程
 */
void Motor_Tick_10ms(Motor_HandleTypeDef *hmotor) {
    // --- 感知层 ---
    Motor_Update_Encoder(hmotor);
    // --- 决策层：PID计算并输出PWM
    Motor_Calculate_Control(hmotor);
}

/* 定时器溢出中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        Sensor_Global_Scan(&tracker);
        if (car_mode == MODE_STANDBY) //鸣笛处理，这个对称美，就放着了
        {
            void alarm();
        }
        if (car_mode == MODE_LINE_TRACKING) //这个要严格控制10ms执行一次所以放在这判断
        {
            Track_line(&tracker,&MotorA, &MotorB);  //循迹模式
        }
        if (car_mode == MODE_BACKING) //这个要严格控制10ms执行一次所以放在这判断
        {
            Back_line(&tracker);
        }
        Motor_Tick_10ms(&MotorA);  //电机A pwm输出
        Motor_Tick_10ms(&MotorB);  //电机B pwm输出
    }
}