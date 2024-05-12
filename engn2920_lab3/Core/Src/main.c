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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define X_TABLE_SIZE 500
#define Y_TABLE_SIZE 500

/*
#define x_on HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 0)
#define x_off HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 1)
#define x_move_left HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 1)
#define x_move_rigjt HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 0)
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
signed int x_pos=0;
signed int y_pos=0;
int arr_X[X_TABLE_SIZE];
int arr_Y[Y_TABLE_SIZE];

volatile bool X_moveCompleted = false;
volatile bool Y_moveCompleted = false;

int X_step_count = 0; //global
int Y_step_count = 0; //global
int X_moving = 0;
int Y_moving = 0;
int X_step_goal = 0;
int Y_step_goal = 0;
int X_period = 0;
int Y_period = 0;
int x_i;
int y_i;

int pen_is_down = 0;
int pen_is_up = 0;
uint32_t pen_pressure= 0;

static uint32_t gper=0;

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

void generate_lookup_table(int *arr, int table_size) {
    float t_max = 0.75; // Maximum time to reach full speed
    int v_min= 2000;
	int v_max= 16000;
    float t = 0;
    int f = v_min;
    for (int i = 0; i < table_size; i++) {
        if (t < t_max) {
            t += (float)1/f;
            f = ((v_max - v_min) * t / t_max) + v_min;
            arr[i] = 64000000 / f;  // Calculate timer reload value based on frequency
        }
    }
}

int min(int a, int b) {
    return (a < b) ? a : b;
}

float x_ratio = 1.0;
float y_ratio = 1.0;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
   if (htim == &htim2 && X_moving)
   {
	   /*
		   float x_ratio = 1.0;
			   if (Y_step_goal > X_step_goal){
				   x_ratio = (Y_step_goal / X_step_goal);
				   int k = X_step_count * x_ratio;
				   x_i=  min(min(k, X_step_goal - k), X_TABLE_SIZE-1);
				   X_period = x_ratio * arr_X[x_i] - 1;
			   }else{
				   x_i = min(min(X_step_count, X_step_goal - X_step_count-1), X_TABLE_SIZE-1);
				   X_period = arr_X[x_i] - 1;
			   }
			   */

	   /*
	   if (Y_step_goal > X_step_goal){
		   x_ratio = (Y_step_goal / X_step_goal);
	   	   	   X_period = arr_X[100]*x_ratio  - 1;
	   }else{
		   //x_ratio=1.0;
		   //X_period = arr_X[100] - 1;
	   }
  			   TIM2->ARR = X_period-1;
  			   TIM2->CCR4 = X_period/2-1;

  			   */

		   if (X_step_count >= X_step_goal){
		   			  X_moving = 0;
		   			  HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 1);
		   			  TIM2->ARR = 0;
		   			  HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_4);
		   			  X_moveCompleted= true;
		   			  myprintf("DONE moving X %d\n\r",X_step_count);

		   			 Y_moving = 0;
				  HAL_GPIO_WritePin(Y_enable_actlo_GPIO_Port, Y_enable_actlo_Pin, 1);
				  TIM3->ARR = 0;
				  HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
				  Y_moveCompleted = true;
				  myprintf("DONE moving Y %d\n\r",Y_step_goal);
		   		   }
		   X_step_count++;//=x_ratio;
   }

   if (htim == &htim3 && Y_moving)
     {
  			   /*
  			   if ( X_step_goal > Y_step_goal){
  				   y_ratio= (X_step_goal/Y_step_goal);
				   int k = Y_step_count * y_ratio;
				   y_i=  min(min(k, Y_step_goal  - k), Y_TABLE_SIZE-1);
				   //myprintf("y k= %d,  i= %d",k, y_i);
  			   }else{
  				 y_ratio=1.0;
  				   y_i = min(min(Y_step_count, Y_step_goal - Y_step_count-1), Y_TABLE_SIZE-1);
  			   }
  			   */
/*
	   	   	   	   if ( X_step_goal > Y_step_goal){
	     				   y_ratio= (X_step_goal/Y_step_goal);
	     				   Y_period = arr_Y[100]/y_ratio  - 1;
	   				   //myprintf("y k= %d,  i= %d",k, y_i);
	     			   }else{
	     				 y_ratio=1.0;
	     				 Y_period = arr_Y[100] - 1;
	     				  // y_i = min(min(Y_step_count, Y_step_goal - Y_step_count-1), Y_TABLE_SIZE-1);
	     			   }
	     			    	   	 TIM3->ARR = Y_period-1;
	   	  TIM3->CCR1 = Y_period/2-1;
*/
/*
  			   if(!gper){
  				   gper = y_ratio * arr_Y[100];
  				   TIM3->CCR1 = 1000-1;
  			   }else{
  				   TIM3->CCR1 = 0;
  			   }
  			   uint32_t u = min(gper>>(gper >= 65536),65536);
  			   TIM3->ARR = u-1;
  			   gper -= u;
  			    	   	 TIM3->ARR = Y_period-1;
	   	  TIM3->CCR1 = Y_period/2-1;

  			   */

/*
  			 if (Y_step_count/y_ratio >= Y_step_goal){
  			   			  Y_moving = 0;
  			   			  HAL_GPIO_WritePin(Y_enable_actlo_GPIO_Port, Y_enable_actlo_Pin, 1);
  			   			  TIM3->ARR = 0;
  			   			  HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
  			   			  Y_moveCompleted = true;
  			   			  myprintf("DONE moving Y %d\n\r",Y_step_goal);
  			 }

  		 Y_step_count++;//y_ratio;
  		 */

     }

   if(htim == &htim4)
   {
	   // if pen up, move till adc #>
	   if (pen_pressure > 10000){
		   pen_is_down=1;
	   }

	   if(pen_pressure < 500 ){ //// if move pen down, move till adc #<
		   pen_is_up=1;
	   }
   }
}


void step_motor_X(int M, int x_dir){
	myprintf("X -> dir =%d, steps= %d %d\n\r",x_dir, M);
	X_moving = 1;
	X_step_goal = M;
	X_step_count = 0;
	x_i=0;

	TIM2->ARR = arr_X[0]-1; //write period for next cycle
	TIM2->CCR4 = 1000 ;

	HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 0);
	HAL_GPIO_WritePin(X_direction_GPIO_Port, X_direction_Pin, x_dir);

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4); // Start PWM
}

void step_motor_Y(int M,int y_dir){
	myprintf("Y -> dir =%d, steps= %d \n\r",y_dir, M);
	Y_moving = 1;
	Y_step_goal = M;
	Y_step_count = 0;
	y_i=0;

	TIM3->ARR = arr_Y[0]-1; //write period for next cycle
	TIM3->CCR1 = 1000 ;

	HAL_GPIO_WritePin(Y_enable_actlo_GPIO_Port, Y_enable_actlo_Pin, 0);
	HAL_GPIO_WritePin(Y_direction_GPIO_Port, Y_direction_Pin, y_dir);

	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); // Start PWM
}

void step_motor_XY(int M_x,int x_dir,int M_y,int y_dir){
		myprintf("step XY [%d, %d]\n\r",M_x,M_y);
		X_step_goal = M_x;
		X_step_count=0;

		float r = M_x/M_y;

		TIM2->ARR = r*arr_X[300]-1; //write period for next cycle
		TIM2->CCR4 = arr_X[0]/2 ;

		HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 0);
		HAL_GPIO_WritePin(X_direction_GPIO_Port, X_direction_Pin, x_dir);

		Y_step_goal = M_y;
		Y_step_count = 0;

		TIM3->ARR = arr_Y[300]-1; //write period for next cycle
		TIM3->CCR1 = arr_Y[0]/2 ;

		HAL_GPIO_WritePin(Y_enable_actlo_GPIO_Port, Y_enable_actlo_Pin, 0);
		HAL_GPIO_WritePin(Y_direction_GPIO_Port, Y_direction_Pin, y_dir);

		X_moving = 1;
		Y_moving = 1;

		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); // Start PWM
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4); // Start PWM
}


void waitForCompletion(){
	while(!X_moveCompleted || !Y_moveCompleted){
		//wait
	}
}

void move_X(int dir){
	TIM2->ARR = 10000-1; //write period for next cycle
	TIM2->CCR4 = 1000 ;

	HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 0);
	HAL_GPIO_WritePin(X_direction_GPIO_Port, X_direction_Pin, dir);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // Start PWM
}

void move_Y(int dir){
	TIM3->ARR = 10000-1; //write period for next cycle
	TIM3->CCR1 = 1000 ;

	HAL_GPIO_WritePin(Y_enable_actlo_GPIO_Port, Y_enable_actlo_Pin, 0);
	HAL_GPIO_WritePin(Y_direction_GPIO_Port, Y_direction_Pin, dir);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Start PWM
}


void moveTo(int x, int y) {
	X_moveCompleted = false;
	Y_moveCompleted = false;

	int dx = (x - x_pos);
	int dy = (y - y_pos);
    // 控制电机移动到(x, y)
    // Function to convert x, y to motor steps and send commands to the motors
    // Convert x, y to steps
    int xSteps = abs(dx)*10;//convertToSteps(abs(dx));
    int ySteps = abs(dy)*10;
    int sx = dx > 0 ? 0 : 1 ;
    int sy = dy > 0 ? 1 : 0 ;

   //step_motor_X(xSteps,sx);
   //step_motor_Y(ySteps,sy);
    step_motor_XY(xSteps,sx,ySteps,sy);

    waitForCompletion();
    x_pos = x;
    y_pos = y;
    X_step_goal = 0;
    Y_step_goal = 0;
    //myprintf("Current position is, [%d,%d]\n\r",x_pos,y_pos);
}

int convertToSteps(int distance){
	int K= 10;//pulses/ revolution
	int L= 1 ;//cm/ revoltuion;
	return distance*K/L;
}

void pen_down(){
	TIM4->ARR = 320000-1; //write period for next cycle
	TIM4->CCR4 = 1000 ;

	HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 0);
	HAL_GPIO_WritePin(Z_direction_GPIO_Port, Z_direction_Pin, 0);

	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4); // Start PWM

	while(!pen_is_down){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		pen_pressure= HAL_ADC_GetValue(&hadc1);
	}

	  HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 1);
	  TIM2->ARR = 0;
	  TIM4->CCR4 = 0;
	  HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);
	  pen_is_up=0;

}

void pen_up(){
	TIM4->ARR = 320000-1; //write period for next cycle
	TIM4->CCR4 = 1000 ;

	HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 0);
	HAL_GPIO_WritePin(Z_direction_GPIO_Port, Z_direction_Pin, 1);

	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4); // Start PWM

	while(!pen_is_up){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		pen_pressure= HAL_ADC_GetValue(&hadc1);
	}

  HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 1);
  TIM2->ARR = 0;
  TIM4->CCR4 = 0;
  HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);
  pen_is_down=0;

}

void drawCircle(int radius) {
    int steps = 1000;//STEPS_PER_REVOLUTION;  // 用全圈的步数来决定细分度
    double PI=3.1415926;
    //double angle =1.0;
    for (int i = 0; i < steps; i++) {
        double angle = 2 * PI * i / steps;
        int x = 0+radius * cos(angle);
        int y = 0+radius * sin(angle);
        moveTo(x, y);
    }
}

void drawStar(int radius) {
	double angle =1.0;
	double PI=3.1415926;
    for (int i = 0; i < 5; i++) {
    	 angle = PI/2 +( i* (2*PI)/5);
		int x = radius * cos(angle);
		int y = radius * sin(angle);
		moveTo(x, y);
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

  /* Reset of all peripheprals, Initializes the Flash interface and the Systick. */
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  myprintf("--Program Start here \n\r");

  int key=0;
  generate_lookup_table(arr_X, X_TABLE_SIZE);
  generate_lookup_table(arr_Y, Y_TABLE_SIZE);

  //step_motor_X(-1000,0);
  //move_motor_Y(1000,1);

 //moveTo(500,300);
  drawStar(10);
 //waitForCompletion();

 //moveTo(0,0);
//drawCircle(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  key= mygetch();
	  if (key=='a'){ //left
		  myprintf("left\n\r");
		  move_X(1);
	  }else if (key=='d'){ // right

		  myprintf("right\n\r");
		  move_X(0);
	  }else if (key=='w'){ // up// back
		  myprintf("up\n\r");
		  move_Y(1);
	  }else if (key=='x'){ // down// front
		  myprintf("down\n\r");
		  move_Y(0);
	  }else if (key=='p'){ // down// front
		  myprintf("pen down\n\r");
		  pen_down();

		  /*
			TIM4->ARR = 320000-1; //write period for next cycle
			TIM4->CCR4 = 1000 ;

			HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 0);
			HAL_GPIO_WritePin(Z_direction_GPIO_Port, Z_direction_Pin, 0);
d
			HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4); // Start PWM
			*/
	  }else if (key=='u'){
 		  myprintf("pen up\n\r");
 		 pen_up();
 		 /*
 		 	TIM4->ARR = 320000-1; //write period for next cycle
 		 	TIM4->CCR4 = 1000 ;

 		 	HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 0);
 		 	HAL_GPIO_WritePin(Z_direction_GPIO_Port, Z_direction_Pin, 1);

 		 	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4); // Start PWM
 		 	*/
	  }else if(key=='s'){
		  myprintf("stop\n\r");
		  HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, 1);
		  TIM2->ARR = 0;
		  TIM2->CCR4 = 0;
		  HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_4);

		  HAL_GPIO_WritePin(Y_enable_actlo_GPIO_Port, Y_enable_actlo_Pin, 1);
		  TIM3->ARR = 0;
		  TIM3->CCR1 = 0;
		  HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);

		  HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 1);
		  TIM2->ARR = 0;
		  TIM4->CCR4 = 0;
		  HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);
	  }else if(key=='t'){   //// test adc feed back
		  HAL_ADC_PollForConversion(&hadc1, 100);
		  pen_pressure= HAL_ADC_GetValue(&hadc1);
		  myprintf("pressure_z %d\n\r", pen_pressure);
	  }else if (key=='i'){
 		  myprintf("pen up\n\r");
 		 	TIM4->ARR = 320000-1; //write period for next cycle
 		 	TIM4->CCR4 = 1000 ;

 		 	HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 0);
 		 	HAL_GPIO_WritePin(Z_direction_GPIO_Port, Z_direction_Pin, 1);

 		 	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4); // Start PWM
	  }else if (key=='o'){
 		  myprintf("pen down\n\r");
 		 	TIM4->ARR = 320000-1; //write period for next cycle
 		 	TIM4->CCR4 = 1000 ;

 		 	HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, 0);
 		 	HAL_GPIO_WritePin(Z_direction_GPIO_Port, Z_direction_Pin, 0);

 		 	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4); // Start PWM
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Period = 4294967295;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Y_direction_GPIO_Port, Y_direction_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Y_enable_actlo_GPIO_Port, Y_enable_actlo_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(X_direction_GPIO_Port, X_direction_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(X_enable_actlo_GPIO_Port, X_enable_actlo_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Z_enable_actlo_GPIO_Port, Z_enable_actlo_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Z_direction_GPIO_Port, Z_direction_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LX2_limit_right_Pin */
  GPIO_InitStruct.Pin = LX2_limit_right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LX2_limit_right_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Y_direction_Pin Y_enable_actlo_Pin */
  GPIO_InitStruct.Pin = Y_direction_Pin|Y_enable_actlo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : X_direction_Pin X_enable_actlo_Pin */
  GPIO_InitStruct.Pin = X_direction_Pin|X_enable_actlo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LX1_limit_left_Pin */
  GPIO_InitStruct.Pin = LX1_limit_left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LX1_limit_left_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Z_enable_actlo_Pin */
  GPIO_InitStruct.Pin = Z_enable_actlo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Z_enable_actlo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LY2_limit_bot_Pin LY1_limit_top_Pin */
  GPIO_InitStruct.Pin = LY2_limit_bot_Pin|LY1_limit_top_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Z_direction_Pin */
  GPIO_InitStruct.Pin = Z_direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Z_direction_GPIO_Port, &GPIO_InitStruct);

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
