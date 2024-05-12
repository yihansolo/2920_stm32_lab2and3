/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TABLE_SIZE 2000


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void myprintf (const char *fmt, ...)
{
	va_list arglist;
	int nby;
	unsigned char st[256];

	if (!fmt) return;
	va_start(arglist,fmt);
	nby = vsnprintf((char *)st,sizeof(st),fmt,arglist);
	va_end(arglist);
	if (nby < 0) { nby = sizeof(st)-1; } //print truncated string in case too long
	if (nby) HAL_UART_Transmit(&huart3,st,nby,10);
}

int mygetch (void) //Poll
{
	uint8_t ch = 0;
	if (HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, 0) == HAL_OK) return ch;
	return -1; //HAL_TIMEOUT
}



int arr[TABLE_SIZE];

void generate_lookuptalbe(){
	int v_min = 3200;//2000;
	int v_max = 32000;//16000;
	float t_max = 0.75;
	int f = 0;
	float t = 0;

	f = v_min;
	for (int i=0;i<TABLE_SIZE;i++){
		if (t<t_max){
			t +=(float)1/f;
			f = ((v_max-v_min)*t/t_max)+v_min;
			arr[i]=64000000/f;
		}
	}
	myprintf("look up table generated\n\r");
}

int min(int a, int b) {
    return (a < b) ? a : b;
}

int calibration_active = 0;
int calibration_step_count = 0;
int calibration_stage = 0;
int calibration_check=0;

int step_count=0; //global
int moving = 0;
int step_goal=0;
int i=0;
int dir=0;
int L=0;
int R=0;
int period=0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
   if (htim == &htim2)
   {
	   if(calibration_active){  // Calibration
		   calibration_step_count++;
		   R = HAL_GPIO_ReadPin(R_switch_GPIO_Port,R_switch_Pin);
		   L = HAL_GPIO_ReadPin(L_switch_GPIO_Port,L_switch_Pin);
		   if (L == 0 ){  // stage 0 and stage 2
			   myprintf("L= %d, R= %d\n\r",L, R);
			   myprintf("CALIBRATION stage %d \n\r",calibration_stage);
			   HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 1);
			   TIM2->ARR = 0;
			   dir= 1;
			   if(calibration_stage == 0){
				   calibration_stage=1;
				   calibration_step_count=0;
			   }
		   }else if (R==0){ //stage 1 and 3
			   myprintf("L= %d, R= %d\n\r",L, R);
			   myprintf("CALIBRATION stage %d \n\r",calibration_stage);
			   HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 1);
			   TIM2->ARR = 0;
			   dir= 0;

			   if(calibration_stage == 1){
				   calibration_stage=2;
				   //HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 1);
				   calibration_active=0;
				   calibration_check=1;
				   step_count=0;
				   myprintf("CALIBRATION stage %d start\n\r",calibration_stage);
			   }
		   }
		   HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, dir);
		   HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 0);
		   TIM2->ARR = 6000-1;
	   }

	   if(calibration_check){
		   step_count++;
		   if (step_count >= calibration_step_count/2){
			   calibration_check = 0;
				  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 1);
				  TIM2->ARR = 0;
				  HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_4);
				  myprintf("CALIBRATION DONE %d\n\r",calibration_step_count);
			   }else{ // update period
				   i = min(min(step_count, calibration_step_count - step_count), TABLE_SIZE-1);
				   period = arr[i]-1;
			   }
	   }

	   if(moving){
		   step_count++;

		   if (step_count >= step_goal){
			  moving=0;
			  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 1);
			  TIM2->ARR = 0;
			  HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_4);
			  myprintf("DONE moving %d\n\r",step_goal);
		   }else{ // update period
			   i = min(min(step_count, step_goal - step_count), TABLE_SIZE-1);
			   period = arr[i]-1;
		   }

		   R = HAL_GPIO_ReadPin(R_switch_GPIO_Port,R_switch_Pin);
		   L = HAL_GPIO_ReadPin(L_switch_GPIO_Port,L_switch_Pin);
		   if (L == 0 ){
			   HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 1);
			   TIM2->ARR = 0;
			   dir = 1;
		   }else if (R==0){
			   HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 1);
			   TIM2->ARR = 0;
			   dir = 0;
		   }
		   HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, dir);
		   HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 0);
		   TIM2->ARR = period-1;
		   TIM2->CCR4 = period/2-1;
	   }
   }
}

void calibrate_motor(){
    calibration_active = 1;
    //calibration_step_count = 0
	myprintf("CALIBRATION \n\r");
	TIM2->ARR = 6000-1; //write period for next cycle
	TIM2->CCR4 = 200 -1;

	HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 0);
	HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, 0);// 0 for left

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4); // Start PWM
}

void move_motor(int M, int dir){
	myprintf("move to the %d for %d\n\r",dir, M);
	moving = 1;
	step_goal=M;

	step_count=0;
	TIM2->ARR = arr[0]-1; //write period for next cycle
	TIM2->CCR4 = arr[0]/2 ;

	HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, 0);
	HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, dir);

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4); // Start PWM

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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  myprintf("--Program Start \n\r");
  calibrate_motor();
  generate_lookuptalbe();
  //move_motor(163917/2,1);

  if(calibration_check){
	  move_motor(calibration_step_count/2,1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // calibration // 163917  166600 loop==2

  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : L_switch_Pin */
  GPIO_InitStruct.Pin = L_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(L_switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R_switch_Pin */
  GPIO_InitStruct.Pin = R_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(R_switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Direction_Pin */
  GPIO_InitStruct.Pin = Direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Direction_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Enable_Pin */
  GPIO_InitStruct.Pin = Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Enable_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

#ifdef  USE_FULL_ASSERT
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
