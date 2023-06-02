/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pitch, roll, yaw;    // 欧拉角
short aacx, aacy, aacz;    // 加速度传感器原始数据
short gyrox, gyroy, gyroz; // 陀螺仪原始数据
float ax, ay, az;
float temp; // 温度
int topSurface = 0;

short last_ax, last_ay, last_az; // 用来保存上次的加速度数据
float threshold = 1.75;          // 阈值
int output_on = 0;

uint8_t g_ucUsart1ReceiveData;

int mode[5] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void topSurfaceIdentify(void);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU_Init();     // MPU6050初始化
  mpu_dmp_init(); // dmp初始化
  printf("初始化成功！\r\n");
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart1, &g_ucUsart1ReceiveData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(500);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // topSurfaceIdentify();
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  last_ax = ax;
  last_ay = ay;
  last_az = az;
  if (htim == (&htim1))
  {
    while (mpu_dmp_get_data(&pitch, &roll, &yaw))
      ;                                         // 必须要用while等待，才能读取成功
    MPU_Get_Accelerometer(&aacx, &aacy, &aacz); // 得到加速度传感器数据
    ax = (float)aacx / 16384.0;
    ay = (float)aacy / 16384.0;
    az = (float)aacz / 16384.0;
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz); // 得到陀螺仪数据
    // 计算加速度的变化
    float delta_aacx = fabsf(ax - last_ax);
    float delta_aacy = fabsf(ay - last_ay);
    float delta_aacz = fabsf(az - last_az);
    float total_delta = sqrtf(delta_aacx * delta_aacx + delta_aacy * delta_aacy + delta_aacz * delta_aacz);
    // temp = MPU_Get_Temperature();               // 得到温度信息

    // printf("data:%.1f,%.1f,%.1f\r\n", ax, ay, az); // 串口1输出采集信息

    // 检测是否有摇动
    if (mode[0] == 1)
    {
      if (total_delta > threshold)
      {
        output_on = !output_on;
        if (output_on)
        {
          printf("ON\r\n");
        }
        else
        {
          printf("OFF\r\n");
        }
      }
    }
  }

  if (mode[1] == 1)
  {
    printf("data:%.1f,%.1f,%.1f\r\n", roll, pitch, yaw); // 串口1输出采集信息
  }

  if (mode[2] == 1)
  {
    topSurfaceIdentify();
  }
}
void topSurfaceIdentify()
{
  if (roll > -30 && roll < 30)
  {
    // topSurface = 4; // 顶面为E
    printf("topSurface is E\r\n");
  }
  else if (roll > 60 && roll < 120)
  {
    printf("topSurface is D\r\n");
  }
  else if ((roll < 180 && roll > 150) || (roll < -150 && roll > -180))
  {
    // topSurface = 5; // 顶面为F
    printf("topSurface is F\r\n");
  }
  else
  {
    if (pitch > 60)
    {
      // topSurface = 0; // 顶面为A
      printf("topSurface is A\r\n");
    }
    else if (pitch > -30 && pitch < 30)
    {
      printf("topSurface is B\r\n");
    }
    else if (pitch < -60)
    {
      printf("topSurface is C\r\n");
    }
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
