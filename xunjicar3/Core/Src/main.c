/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "control.h"
#include "oled.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint32_t last_tick = 0; //oled 调试信息上次打印的时间
uint8_t display_buf[64]; //显示信息缓冲区
uint32_t last_vision_time = 0; //上一次视觉识别时间
int err_x = 0, err_y = 0;
int distance = 0;
#define RX_MAX_LEN 128  //串口dma中断
char rx_buffer[RX_MAX_LEN + 1]; //串口dma中断
uint8_t rx_flag = 0; //串口dma中断
uint16_t rx_packet_len = 0; //串口dma中断

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "System Init...", 12);

    Motor_Init(&MotorA,
               GPIOB, GPIO_PIN_2,
               GPIOB, GPIO_PIN_10,
               &htim4, TIM_CHANNEL_1,
               &htim2);

    Motor_Init(&MotorB,
               GPIOB, GPIO_PIN_1,
               GPIOB, GPIO_PIN_0,
               &htim4, TIM_CHANNEL_2,
               &htim3);
    Car_Strategy_Init();

    HAL_TIM_Base_Start_IT(&htim10);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)rx_buffer, RX_MAX_LEN);

    OLED_ShowString(0, 1, "Ready to Go!", 12);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

        static uint8_t key_lock = 0;

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) // 检测到按键按下
        {
            if (key_lock == 0)
            {
                HAL_Delay(20); // 软件消抖
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
                {
                    // 核心逻辑：在 TRACKING 和 FINISHED 之间来回翻转
                    if (Current_Stop_State == STATE_TRACKING)
                    {
                        Current_Stop_State = STATE_FINISHED;
                        finish_camera_task = 1;

                        // 【安全保护】：强行切到完成状态时，立刻让电机断电停止
                        Car_Set_Speed(&MotorA, 0);
                        Car_Set_Speed(&MotorB, 0);
                    }
                    else
                    {
                        // 重新开始循迹
                        car_mode = MODE_LINE_TRACKING;      // 确保主模式正确
                        Current_Stop_State = STATE_TRACKING; // 切换回循迹阶段
                        Car_Strategy_Init();
                        distance = 0;
                        finish_camera_task = 0;
                    }

                    key_lock = 1; // 锁死按键
                }
            }
        }
        else
        {
            key_lock = 0; // 手松开，解锁
        }


        // 视觉超时保护，100ms 没收到有效视觉包，自动退回循迹
        if (car_mode == MODE_VISION_FOLLOW)
        {
            if (HAL_GetTick() - last_vision_time > 100)
            {
                car_mode = MODE_LINE_TRACKING;
                Car_Set_Speed(&MotorA, 0);
                Car_Set_Speed(&MotorB, 0);
            }
        }

        // 视觉模式控制


       if (rx_flag == 1)
        {
            // === 【核心优化 1】：引入影子缓冲区，保护活体 DMA 内存 ===
            static char parse_buffer[RX_MAX_LEN + 1];

            uint16_t len = rx_packet_len;
            if (len > RX_MAX_LEN) len = RX_MAX_LEN;

            // 瞬间把数据捞出来，不给 DMA 干扰的机会
            memcpy(parse_buffer, rx_buffer, len);
            parse_buffer[len] = '\0';

            // 捞完数据立刻允许接收下一帧，且绝对不要对 rx_buffer 动用 memset！
            rx_flag = 0;

            // 局部变量初始化
            int local_err_x = 0, local_err_y = 0, local_distance = 0;

            // === 【核心优化 2】：推荐在 OpenMV 端将发送格式改为 "#%d,%d,%d\r\n" ===
            // 如果 OpenMV 端加了 '#' 帧头，这里改成 sscanf(parse_buffer, "#%d,%d,%d", ...)
            int matched = sscanf(parse_buffer, "#%d,%d,%d", &local_err_x, &local_err_y, &local_distance);

            if (matched == 3)
            {
                // 数据合法性刚性过滤（根据 OpenMV 实际范围加一道防火墙）
                // 既然 OpenMV 只发 20~100，任何超出这个范围的数据直接丢弃！
                if (local_distance >= 10 && local_distance <= 110)
                {
                    // 校验通过，再赋值给全局变量
                    err_x = local_err_x;
                    err_y = local_err_y;
                    distance = local_distance;

                    if (finish_camera_task == 0)
                    {
                        if (car_mode == MODE_LINE_TRACKING && distance <= STOP_DISTANCE)
                        {
                            car_mode = MODE_VISION_FOLLOW;
                        }

                        if (car_mode == MODE_VISION_FOLLOW)
                        {
                            last_vision_time = HAL_GetTick();

                            // 你的刚性限幅保持
                            if (distance >= 100) distance = 100;

                            track_camera((float)err_x, (float)distance);
                        }
                    }
                }
            }
            // 【核心优化 3】：删除了原先的 memset(rx_buffer, 0, RX_MAX_LEN);
        }


        //显示信息
        if (HAL_GetTick() - last_tick >= 500)
        {
            last_tick = HAL_GetTick();
            sprintf((char*)display_buf, "dist:%d     ", distance);
            OLED_ShowString(0, 6, display_buf, 16);
            sprintf((char*)display_buf, "Lt%.2f;Rt%.2f", MotorA.speed_target, MotorB.speed_target);
            OLED_ShowString(0, 0, display_buf, 12);

            sprintf((char*)display_buf, "Lr%.2f;Rr%.2f", MotorA.speed, MotorB.speed);
            OLED_ShowString(0, 1, display_buf, 12);


            if (car_mode == MODE_LINE_TRACKING)
            {
                OLED_ShowString(0, 2, (uint8_t*)"Mode: TRACKING ", 12);
            }
            if (car_mode == MODE_VISION_FOLLOW)
            {
                OLED_ShowString(0, 2, (uint8_t*)"Mode: VISION   ", 12);
            }
            if (car_mode == MODE_STANDBY)
            {
                OLED_ShowString(0, 2, (uint8_t*)"Mode: STANDBY   ", 12);
            }
            if (car_mode == MODE_BACKING)
            {
                OLED_ShowString(0, 2, (uint8_t*)"Mode: BACKING   ", 12);
            }

            if (Current_Stop_State == STATE_TRACKING)
            {
                OLED_ShowString(0, 3, (uint8_t*)"Mode1", 12);
            }
            if (Current_Stop_State == STATE_CROSSING_BAR)
            {
                OLED_ShowString(0, 3, (uint8_t*)"Mode2", 12);
            }
            if (Current_Stop_State == STATE_MOVE_TO_GARAGE)
            {
                OLED_ShowString(0, 3, (uint8_t*)"Mode3", 12);
            }
            if (Current_Stop_State == STATE_FINISHED)
            {
                OLED_ShowString(0, 3, (uint8_t*)"Mode4", 12);
            }
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}


/**
 * @brief  DMA + 串口空闲中断回调函数
 * @param  huart
 * @param  Size: DMA 硬件自动统计出的这一帧文字的总字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // 健壮性保护：如果异常数据超过了设定的最大长度，强行截断，防止打爆内存
        if (Size > RX_MAX_LEN) { Size = RX_MAX_LEN; }
        rx_packet_len = Size;
        rx_buffer[Size] = '\0';

        rx_flag = 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)rx_buffer, RX_MAX_LEN);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)rx_buffer, RX_MAX_LEN);
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
