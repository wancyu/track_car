#include "speed.h"
#include "motor.h"
#include "tim.h"
#include "track.h"

int16_t g_left_r_speed;
int16_t g_right_r_speed;
extern int16_t g_left_t_speed;
extern int16_t g_right_t_speed;

// ================= PID参数 =================
float kp = 12.0f;
float ki = 0.1f;
float kd = 2.5f;

// ================= 左电机 =================
float err_l = 0;
float last_err_l = 0;
float integ_l = 0;

// ================= 右电机 =================
float err_r = 0;
float last_err_r = 0;
float integ_r = 0;

// ================= 限幅 =================
static int16_t limit_pwm(int16_t val)
{
    if (val > 7200) return 7200;
    if (val < -7200) return -7200;
    return val;
}

// ================= 读取左编码器 =================
static int16_t GetLeftEncoder(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return cnt;
}

// ================= 读取右编码器 =================
static int16_t GetRightEncoder(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return cnt;
}

// ================= 左PID =================
static int16_t PID_Left(int16_t target, int16_t actual)
{
    err_l = target - actual;

    integ_l += err_l;

    // 积分限幅
    if (integ_l > 10000) integ_l = 10000; // 给积分项留出足够的“补油”空间
    if (integ_l < -10000) integ_l = -10000;

    float d = err_l - last_err_l;

    float out = kp * err_l + ki * integ_l + kd * d;

    last_err_l = err_l;

    return (int16_t)out;
}

// ================= 右PID =================
static int16_t PID_Right(int16_t target, int16_t actual)
{
    err_r = target - actual;

    integ_r += err_r;

    if (integ_r > 10000) integ_r = 10000;
    if (integ_r < -10000) integ_r = -10000;

    float d = err_r - last_err_r;

    float out = kp * err_r + ki * integ_r + kd * d;

    last_err_r = err_r;

    return (int16_t)out;
}

// ================= 主控制函数 =================
void Speed_Control(int16_t target_left, int16_t target_right)
{
    // 读取实际速度（编码器）
    int16_t actual_left = GetLeftEncoder();
    int16_t actual_right = GetRightEncoder();

    g_left_r_speed = actual_left;
    g_right_r_speed = actual_right;

    // PID计算
    int16_t pwm_left = PID_Left(target_left, actual_left);
    int16_t pwm_right = PID_Right(target_right, actual_right);

    // 输出到电机
    Load(limit_pwm(pwm_left), limit_pwm(pwm_right));
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 检查是不是定时器9触发的中断 (2ms 一次)
    if (htim->Instance == TIM9)
    {
        switch (car_mode)
        {
            case MODE_TRACK:
                Tracking_PD(); // 原本的循迹闭环
                break;

            case MODE_PARK_DRIVE:
                // 进库盲走，给一个恒定速度
                g_left_t_speed = 800;
                g_right_t_speed = 800;
                action_timer++;
                // 盲走时间：假设定时器是 2ms，走 0.5 秒就是 250 次
                if (action_timer > 250) {
                    car_mode = MODE_STOP;
                }
                break;

            case MODE_CAMERA:
                Camera_PD(); // 摄像头对准与接近
                break;

            case MODE_WAIT:
                g_left_t_speed = 0;
                g_right_t_speed = 0;
                action_timer++;
                // 停车等待 3 秒：1500 * 2ms = 3000ms
                if (action_timer > 1500) {
                    car_mode = MODE_REVERSE;
                    action_timer = 0;

                    // 倒车前清空电机 PID 的积分，防止窜动
                    extern float integ_l, integ_r;
                    integ_l = 0; integ_r = 0;
                }
                break;

            case MODE_REVERSE:
                // 倒车，给定负速度
                g_left_t_speed = -1000;
                g_right_t_speed = -1000;
                action_timer++;
                // 倒车时间：比如倒车 2 秒 = 1000 次
                if (action_timer > 1000) {
                    car_mode = MODE_TRACK; // 倒车完毕，恢复循迹
                    action_timer = 0;

                    // 同样清一下误差
                    extern float last_error;
                    last_error = 0;
                    extern float integ_l, integ_r;
                    integ_l = 0; integ_r = 0;

                    // 强制清空串口距离，防止刚切回循迹又被摄像头打断
                    cam_dist = 100;
                }
                break;

            case MODE_STOP:
                g_left_t_speed = 0;
                g_right_t_speed = 0;
                // 彻底停死，不再做任何动作
                break;
        }

        // 不管处于什么模式，统一将计算出的目标速度送入底层电机闭环
        Speed_Control(g_left_t_speed, g_right_t_speed);
    }
}