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
uint8_t display_buf[20];  //显示信息缓冲区
uint32_t last_vision_time = 0; //上一次视觉识别时间，这个不应该放在这？

#define RX_MAX_LEN 128  //串口dma中断
char rx_buffer[RX_MAX_LEN+1]; //串口dma中断
uint8_t  rx_flag = 0; //串口dma中断
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
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "System Init...", 12);

  Motor_Init(&MotorA,
             GPIOB, GPIO_PIN_0,
             GPIOB, GPIO_PIN_1,
             &htim4, TIM_CHANNEL_1,
             &htim2);

  Motor_Init(&MotorB,
             GPIOB, GPIO_PIN_10,
             GPIOB, GPIO_PIN_2,
             &htim4, TIM_CHANNEL_2,
             &htim3);
  Car_Strategy_Init();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rx_buffer, RX_MAX_LEN);
  HAL_TIM_Base_Start_IT(&htim10);
  OLED_ShowString(0, 1, "Ready to Go!", 12);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 视觉超时保护，100ms 没收到有效视觉包，自动退回循迹
    if (car_mode == MODE_VISION_FOLLOW)
    {
      if (HAL_GetTick() - last_vision_time > 100)
      {
        car_mode = MODE_LINE_TRACKING;
      }
    }

    // 视觉模式控制
    if (finish_camera_task == 0)
    {
      if (rx_flag == 1)
      {
        int err_x = 0, err_y = 0, distance = 0;

        // 尝试解析标准的 逗号 分隔符
        int matched = sscanf(rx_buffer, "%d,%d,%d", &err_x, &err_y, &distance);

        if (matched == 3)
        {
          car_mode = MODE_VISION_FOLLOW;
          last_vision_time = HAL_GetTick();

          // 激活外环控制逻辑
          printf("Parsed OK: %d, %d, %d\n", err_x, err_y, distance);
          track_camera((float)err_x, (float)distance);
        }
        else
        {
          //
        }
        rx_flag = 0;
        memset(rx_buffer, 0, RX_MAX_LEN);
      }
    }

    //显示信息
    if (HAL_GetTick() - last_tick >= 100)
    {
      last_tick = HAL_GetTick();
      sprintf((char *)display_buf, "Lt%.2f;Rt%.2f", MotorA.speed_target,MotorB.speed_target);
      OLED_ShowString(0, 2, display_buf, 12);
      sprintf((char *)display_buf, "Rr%.2f;Rr%.2f", MotorA.speed,MotorB.speed);
      OLED_ShowString(0, 3, display_buf, 12);
      if (car_mode == MODE_LINE_TRACKING) {
        OLED_ShowString(0, 0, (uint8_t *)"Mode: TRACKING ", 16);
      } else {
        OLED_ShowString(0, 0, (uint8_t *)"Mode: VISION   ", 16);
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    // 健壮性保护：如果异常数据超过了设定的最大长度，强行截断，防止打爆内存
    if (Size > RX_MAX_LEN) {Size = RX_MAX_LEN;}
    rx_packet_len = Size;
    rx_buffer[Size] = '\0';

    rx_flag = 1;
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rx_buffer, RX_MAX_LEN);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rx_buffer, RX_MAX_LEN);
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
