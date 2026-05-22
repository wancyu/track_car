#include "track.h"
#include "motor.h"
#include "gpio.h"
#include "usart.h"
// ================= 全局变量定义 =================
int16_t g_left_t_speed = 0;
int16_t g_right_t_speed = 0;
int s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0;
CarMode car_mode = MODE_TRACK; // 默认启动为循迹模式
uint32_t action_timer = 0;     // 动作计时器
float cam_err = 0;             // 摄像头误差
float cam_dist = 100;          // 默认距离给大一点

static uint8_t garage_flag = 0; // 记录是否压过车库线的标志位
// ================= 内部静态变量 =================
static float last_error = 0;
static uint8_t lost_cnt = 0;

// ================= 随速映射配置 =================
#define STANDARD_SPEED 1000.0f  // 更改基准速度为 1000
#define BASE_KP        300.0f   // 在 1000 速度下，先给一个保守的 KP
#define BASE_KD        450.0f    // 在 1000 速度下，先给一个保守的 KD

#define BASE_SPEED     1000     // 当前实测基础速度
#define MAX_PWM        7200
#define LOST_LIMIT     100
/**
 * @brief 限幅函数
 */
static int16_t clamp(int16_t val)
{
    if (val > MAX_PWM) return MAX_PWM;
    if (val < -MAX_PWM) return -MAX_PWM;
    return val;
}

/**
 * @brief 带速度映射的位置环算法
 */
void Tracking_PD(void)
{
    // 1. 读取传感器
    s1 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    s2 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
    s3 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
    s4 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
    s5 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

    int sum = 0, cnt = 0;
    if (s1) { sum += -4; cnt++; }
    if (s2) { sum += -1; cnt++; }
    if (s3) { sum +=  0; cnt++; }
    if (s4) { sum +=  1; cnt++; }
    if (s5) { sum +=  4; cnt++; }

    if (cnt >= 4) {
        garage_flag = 1; // 压到横向黑线（大部分变黑），标记准备入库
    }

    // 如果之前压过黑线，现在全白了，说明车身进库了
    if (garage_flag == 1 && cnt == 0) {
        car_mode = MODE_PARK_DRIVE; // 切换为进库盲走模式
        action_timer = 0;           // 清零计时器
        garage_flag = 0;            // 复位标志位
        return;                     // 直接退出，不走后续的 PD
    }


    // 2. 丢线处理
    if (cnt == 0)
    {
        lost_cnt++;
        if (lost_cnt > LOST_LIMIT)
        {
            g_left_t_speed = 0; g_right_t_speed = 0;
            Load(0, 0); return;
        }
    }
    else
    {
        lost_cnt = 0;
        float error = (float)sum / (float)cnt;

        // --- 核心：随速映射计算 ---
        // 如果 BASE_SPEED 是 1500，ratio 就是 0.5，PD 力度减半，防止低速抖动
        // 如果 BASE_SPEED 提到 3000，ratio 就是 1.0，恢复标准力度
        float speed_ratio = (float)BASE_SPEED / STANDARD_SPEED;
        float current_kp = BASE_KP * speed_ratio;
        float current_kd = BASE_KD * speed_ratio;

        // 3. PD计算
        float p = current_kp * error;
        float d = current_kd * (error - last_error);
        float output = p + d;
        last_error = error;

        // 4. 差速叠加 (修正后的方向)
        int16_t left  = BASE_SPEED + output;
        int16_t right = BASE_SPEED - output;

        g_left_t_speed = clamp(left);
        g_right_t_speed = clamp(right);
    }
}

// 新增：基于摄像头的 PD 控制
void Camera_PD(void)
{
    // 如果距离小于 40cm，触发停车等待逻辑
    if (cam_dist < 40.0f) {
        car_mode = MODE_WAIT;
        action_timer = 0;
        g_left_t_speed = 0;
        g_right_t_speed = 0;
        return;
    }

    // 摄像头 PD 计算 (参数需要根据实际情况调，这里给个保守值)
    float cam_kp = 5.0f;
    float cam_kd = 10.0f;

    float p = cam_kp * cam_err;
    float d = cam_kd * (cam_err - last_error); // 复用之前的 last_error
    float output = p + d;
    last_error = cam_err;

    // 设定一个较低的接近速度
    int16_t approach_speed = 800;

    // 根据误差左右调速，让目标居中
    int16_t left  = approach_speed + output;
    int16_t right = approach_speed - output;

    g_left_t_speed = clamp(left);
    g_right_t_speed = clamp(right);
}

// 假设这是你的串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) // 假设摄像头接在串口2
    {
        // 1. 这里填入你的协议解析代码
        // 解析出误差给 cam_err，距离给 cam_dist
        // 假设已经成功解析：
        // cam_err = 解析到的横向误差;
        // cam_dist = 解析到的距离;

        // 2. 状态切换逻辑：如果在常规循迹时收到数据，立刻切入摄像头模式
        if (car_mode == MODE_TRACK && cam_dist > 0)
        {
            car_mode = MODE_CAMERA;
            // 清理一下历史误差，防止切模式瞬间抽搐
            extern float last_error;
            last_error = 0;
        }

        // 重新开启接收中断...
        // HAL_UART_Receive_IT(...);
    }
}