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

#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>

#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RECEIEVE_DATA_LENGTH 3


char message[100];
char receiveData[RECEIEVE_DATA_LENGTH+1]={'\0'},buffer='\0';
uint16_t next_voltage_mv=0,target_PWM=50;

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

//通信
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if(huart->Instance == USART2)
  {
    //与发信端约定固定长度RECEIEVE_DATA_LENGTH，便于解析信号
    static uint8_t index=0;
    if (index<RECEIEVE_DATA_LENGTH-1)
    {


      receiveData[index]=buffer;
      index++;
    }
    else
    {

      receiveData[index]=buffer;
      index++;

      receiveData[index]='\0';
       target_PWM=atoi(receiveData);

      //限制范围
      if (target_PWM<0)
        target_PWM=0;
      if (target_PWM>100)
        target_PWM=100;
      //指针归零
      index=0;

      //反馈
      sprintf(message,"target_PWM: %d %%\r\n", target_PWM);
      HAL_UART_Transmit_IT(&huart2,message,strlen(message));


    }
    //为下一轮接收准备
    HAL_UART_Receive_IT(&huart2,&buffer,1);
  }

}
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
int value=0;
  float voltage_mv=0;

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  //上机/复位反馈
  HAL_UART_Transmit(&huart2,"DONE\r\n",strlen("DONE"),HAL_MAX_DELAY);
  //第一次接收，确保进中断
  HAL_UART_Receive_IT(&huart2,&buffer,1);

  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    //获取光敏电阻电压
 value=HAL_ADC_GetValue(&hadc1);
uint16_t voltage_mv=((float)(value)*3.3)/4095*1000;

    //利用PID函数获取目标电压（mv），即将输出的PWM（%）,即将输出的电压（mv）

  uint16_t  output_PWM_voltage[3]={0},target_voltage_mv=0,next_PWM=0,next_voltage_mv=0;


    PID_Light(target_PWM,voltage_mv,output_PWM_voltage);
    target_voltage_mv=output_PWM_voltage[0];
    next_PWM=output_PWM_voltage[1];
    next_voltage_mv=output_PWM_voltage[2];



    //改变占空比
   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,next_PWM);

    //每s刷新数据 目标pwm 目标电压 目前电压 即将输出的电压
   // sprintf(message,"target_PWM:%d %% ,target_voltage: %d mV,current Voltage: %d mV,next Voltage: %d mV\r\n",target_PWM,(uint16_t)(target_voltage_mv),(uint16_t)(voltage_mv),next_voltage_mv);
    //目标电压 目前电压 即将输出的电压
    sprintf(message,"%d ,%d,%d\r\n",(uint16_t)(target_voltage_mv),(uint16_t)(voltage_mv),next_voltage_mv);

    HAL_UART_Transmit_IT(&huart2,(uint8_t*)message,strlen(message));

    HAL_Delay(20);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
