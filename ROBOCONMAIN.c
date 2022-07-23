/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

osThreadId defaultTaskHandle;
osThreadId GoStraightHandle;
osThreadId RightTurnHandle;
osThreadId LeftTurnHandle;
osThreadId GoBackHandle;
osThreadId ADCHandle;
/* USER CODE BEGIN PV */
static void SpeedCtrl(void);
static void CONVRTADC_DGT(void);
static void InError_Fuzzy(void);
static void InDError_Fuzzy(void);
void Kp_Fuzzy(void);
void Ki_Fuzzy(void);
void Kd_Fuzzy(void);
static float Min_Fuzzy(float x,float y);
static float Max_Fuzzy(float x,float y);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void StartGoStraight(void const * argument);
void StartRightTurn(void const * argument);
void StartLefTurn(void const * argument);
void StartGoBack(void const * argument);
void StartADC(void const * argument);

/* USER CODE BEGIN PFP */
static uint16_t ADCsensor[10];
static uint32_t	SumADCvalue[10];
static uint32_t AvrgADCvalue[10];
unsigned int Avrg_Ctr=0;
signed int Digitalsensor[10];
float Deviation	=	0;
float Pre_Deviation	=	0;
float Pre_Pre_Deviation	=	0;
float NumberSensor=0;
float SUMDe=0;
float Error=0;
float Pre_Error=0;
float Pre_Pre_Error=0;
float P_Ctrl=0;
float R_Ctrl=0;
float I_Ctrl=0;
float D_Ctrl=0;
float Kp =0;
float Ki =0;
float Kr =0.3;
float Kd =0;
float Out_PID=0;
float Pre_Out_PID=0;
unsigned int CountKr=0;
uint32_t A=0;
unsigned int TurnAround=0;
unsigned int Duty=2800;
//Furry parameter
float In_Err_Value[5]={-200,-100,0,100,200};
float In_DErr_Value[5]={-200,-100,0,100,200};
float err[5];
float derr[5];
float DError=0;
float Rule_Kp[5][5]={{5.2,5,1,1,2},{5.2,4.2,1,1,4.2},{4.2,2,1,1,5.2},{2,1,1,2,5.2},{1,1,1,4,5.2}};
float Rule_Ki[5][5]={{0.025,0.025,0.01,0.01,0.015},{0.025,0.02,0.01,0.01,0.02},{0.02,0.015,0.01,0.01,0.015},{0.015,0.01,0.01,0.015,0.025},{0.01,0.01,0.01,0.02,0.025}};
float Rule_Kd[5][5]={{5.2,5.2,1,1,2},{5.2,4,1,1,4},{4,2,1,1,5.2},{2,1,1,2,5.2},{1,1,1,4,5.2}};
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_ADC_Start_DMA(&hadc1, ADCsensor,10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GoStraight */
  osThreadDef(GoStraight, StartGoStraight, osPriorityLow, 0, 128);
  GoStraightHandle = osThreadCreate(osThread(GoStraight), NULL);

  /* definition and creation of RightTurn */
  osThreadDef(RightTurn, StartRightTurn, osPriorityIdle, 0, 128);
  RightTurnHandle = osThreadCreate(osThread(RightTurn), NULL);

  /* definition and creation of LeftTurn */
  osThreadDef(LeftTurn, StartLefTurn, osPriorityIdle, 0, 128);
  LeftTurnHandle = osThreadCreate(osThread(LeftTurn), NULL);

  /* definition and creation of GoBack */
  osThreadDef(GoBack, StartGoBack, osPriorityIdle, 0, 128);
  GoBackHandle = osThreadCreate(osThread(GoBack), NULL);

  /* definition and creation of ADC */
  osThreadDef(ADC, StartADC, osPriorityIdle, 0, 128);
  ADCHandle = osThreadCreate(osThread(ADC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN4_Pin|IN3_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN4_Pin IN3_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN4_Pin|IN3_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc1)
 {
	uint8_t i;
	for(i=0;i<10;i++)
  	SumADCvalue[i]+=ADCsensor[i];
  	Avrg_Ctr++;
	if(Avrg_Ctr==1000)
	{
	for(i=0;i<10;i++)
	AvrgADCvalue[i]=SumADCvalue[i]/Avrg_Ctr;
	Avrg_Ctr=0;
	for(i=0;i<10;i++)
  	SumADCvalue[i]=0;
	CONVRTADC_DGT();
	NumberSensor=Digitalsensor[0]+Digitalsensor[1]+Digitalsensor[2]+Digitalsensor[3]+Digitalsensor[4]+Digitalsensor[5];
	Digitalsensor[0]=Digitalsensor[0]*1;
	Digitalsensor[1]=Digitalsensor[1]*2;
    Digitalsensor[2]=Digitalsensor[2]*3;
	Digitalsensor[3]=Digitalsensor[3]*4;
	Digitalsensor[4]=Digitalsensor[4]*5;
	Digitalsensor[5]=Digitalsensor[5]*6;
	SUMDe=(Digitalsensor[0]+Digitalsensor[1]+Digitalsensor[2]+Digitalsensor[3]+Digitalsensor[4]+Digitalsensor[5]);
	Pre_Pre_Deviation=Pre_Deviation;
    Pre_Deviation=Deviation;
    Deviation=100*(SUMDe/NumberSensor);
	if(NumberSensor==0)
	Deviation=350;

    HAL_ADC_Stop(hadc1);
	hdma_adc1.Init.Mode = DMA_NORMAL;
	HAL_DMA_Init(&hdma_adc1);
	}

 }
void SpeedCtrl(void)
{
	if(Out_PID<0)
	{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty-R_Ctrl);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Duty-R_Ctrl+Out_PID);
	}
	else
	{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty-R_Ctrl-Out_PID);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Duty-R_Ctrl);
	}
}
void CONVRTADC_DGT(void)
{
	if((AvrgADCvalue[0]<1100))
					Digitalsensor[0]=1;
				else
					Digitalsensor[0]=0;
	if((AvrgADCvalue[1]<400))
	  				Digitalsensor[1]=1;
	  			else
	  				Digitalsensor[1]=0;
	if((AvrgADCvalue[2]<500))
	  				Digitalsensor[2]=1;
	  			else
	  				Digitalsensor[2]=0;
	if((AvrgADCvalue[3]<500))
	  				Digitalsensor[3]=1;
	  			else
	  				Digitalsensor[3]=0;
	if((AvrgADCvalue[4]<400))
	  				Digitalsensor[4]=1;
	  			else
	  				Digitalsensor[4]=0;
	if((AvrgADCvalue[5]<350))
	  				Digitalsensor[5]=1;
	  			else
	  				Digitalsensor[5]=0;
}
void InError_Fuzzy(void)
{
	//-------------------------err--------------------------------
	if(Error<In_Err_Value[0]||Error==In_Err_Value[0])// Error<=-200
	{
		err[0]=1;
		for(int i=1;i<5;i++)
		{
			err[i]=0;
		}
	}
	else
	{
		if(Error<In_Err_Value[1]||Error==In_Err_Value[1])// -200<Error<=-100
		{
			err[0]=(In_Err_Value[1]-Error)/100;
			err[1]=1-err[0];
			err[2]=err[3]=err[4]=0;
		}
		else
		{
			if(Error<In_Err_Value[2]||Error==In_Err_Value[2])//-100<E<=-0
			{
				err[0]=0;
				err[1]=(In_Err_Value[2]-Error)/100;
				err[2]=1-err[1];
				err[3]=err[4]=0;
			}
			else
			{
				if(Error<In_Err_Value[3]||Error==In_Err_Value[3])//0<E<=100
				{
					err[0]=err[1]=0;
					err[2]=(In_Err_Value[3]-Error)/100;
					err[3]=1-err[2];
					err[4]=0;
				}
				else
				{
					if(Error<In_Err_Value[4]||Error==In_Err_Value[4])// 100<E<=200
					{
						err[0]=err[1]=err[2]=0;
						err[3]=(In_Err_Value[4]-Error)/100;
						err[4]=1-err[3];
					}
					else
					{
						for(int i=0;i<4;i++)//E>200
						{
							err[i]=0;
						}
						err[4]=1;
					}
				}

			}
		}
	}
}
void InDError_Fuzzy(void)
{
	DError=Error-Pre_Pre_Error;
			if(DError<In_DErr_Value[0]||DError==In_DErr_Value[0])// E<=-400
		{
			derr[0]=1;
			for(int i=1;i<5;i++)
			{
				derr[i]=0;
			}
		}
		else
		{
			if(DError<In_DErr_Value[1]||DError==In_DErr_Value[1])// -400<E<=-200
			{
				derr[0]=(In_DErr_Value[1]-DError)/200;
				derr[1]=1-derr[0];
				derr[2]=derr[3]=derr[4]=0;
			}
			else
			{
				if(DError<In_DErr_Value[2]||DError==In_DErr_Value[2])//-200<E<=-0
				{
					derr[0]=0;
					derr[1]=(In_DErr_Value[2]-DError)/200;
					derr[2]=1-derr[1];
					derr[3]=derr[4]=0;
				}
				else
				{
					if(DError<In_DErr_Value[3]||DError==In_DErr_Value[3])//0<E<=200
					{
						derr[0]=derr[1]=0;
						derr[2]=(In_DErr_Value[3]-DError)/200;
						derr[3]=1-derr[2];
						derr[4]=0;
					}
					else
					{
						if(DError<In_DErr_Value[4]||DError==In_DErr_Value[4])// 200<E<=400
						{
							derr[0]=derr[1]=derr[2]=0;
							derr[3]=(In_DErr_Value[4]-DError)/200;
							derr[4]=1-derr[3];
						}
						else
						{
							for(int i=0;i<4;i++)//E>200
							{
								derr[i]=0;
							}
							derr[4]=1;
						}
					}

				}
			}
		}
}
float Min_Fuzzy(float x,float y)
{
	if(x>y&&y!=0) return(y);
	else return(x);
}
// ham lay max
float Max_Fuzzy(float x,float y)
{
	if(x<y) return(y);
	else return(x);
}
void Kp_Fuzzy(void)
{
	 int i,j;
	 float sum_Rule_Kp=0,sum_err=0;
	for(i=0;i<5;i++)
	{
		for(j=0;j<5;j++)
		{
			sum_Rule_Kp+=Min_Fuzzy(err[i],derr[j])*Rule_Kp[i][j];
			sum_err+=Min_Fuzzy(err[i],derr[j]);
		}
	}
	Kp=sum_Rule_Kp/sum_err;
	sum_Rule_Kp=0;
	sum_err=0;
}
void Ki_Fuzzy(void)
{
	int i,j;
    float sum_Rule_Ki=0,sum_err=0;
	for(i=0;i<5;i++)
	{
		for(j=0;j<5;j++)
		{
			sum_Rule_Ki+=Min_Fuzzy(err[i],derr[j])*Rule_Ki[i][j];
			sum_err+=Min_Fuzzy(err[i],derr[j]);
		}
	}
	Ki=sum_Rule_Ki/sum_err;
	sum_Rule_Ki=0;
	sum_err=0;
}
void Kd_Fuzzy(void)
{
	int i,j;
    float sum_Rule_Kd=0,sum_err=0;
	for(i=0;i<5;i++)
	{
		for(j=0;j<5;j++)
		{
			sum_Rule_Kd+=Min_Fuzzy(err[i],derr[j])*Rule_Kd[i][j];
			sum_err+=Min_Fuzzy(err[i],derr[j]);
		}
	}
	Kd=sum_Rule_Kd/sum_err;
	sum_Rule_Kd=0;
	sum_err=0;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartGoStraight */
/**
* @brief Function implementing the GoStraight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGoStraight */
void StartGoStraight(void const * argument)
{
  /* USER CODE BEGIN StartGoStraight */
  /* Infinite loop */
  for(;;)
  {
	  if(Avrg_Ctr==1)
	  osThreadSetPriority(ADCHandle,osPriorityBelowNormal);
	  if(SUMDe==0)
	  {
	  osThreadSetPriority(GoBackHandle,osPriorityLow);
	  osThreadSetPriority(GoStraightHandle,osPriorityIdle);
	  }
	  Error=(float)(350)-Deviation;
	  InError_Fuzzy();
	  InDError_Fuzzy();
	  Kp_Fuzzy();
	  Ki_Fuzzy();
	  Kd_Fuzzy();
	  P_Ctrl = Kp*Error;
	  if(CountKr==7)
	  {
	   R_Ctrl=0;
	   CountKr=0;
	   I_Ctrl =0;
	  }
	  R_Ctrl += Kr*fabs(Error);
	  I_Ctrl += Ki*Error;
	  D_Ctrl = Kd*( Error - Pre_Error);
	  Pre_Error=Error;
	  CountKr++;
	  Out_PID= P_Ctrl+I_Ctrl+D_Ctrl;
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_SET);
	  SpeedCtrl();
	  A=0;
	  if((NumberSensor>3&&NumberSensor<6)||(SUMDe>15&&SUMDe<22)||TurnAround==1)
	  {

			  osThreadSetPriority(RightTurnHandle,osPriorityLow);
			  osThreadSetPriority(GoStraightHandle,osPriorityIdle);

	  }

  }
  /* USER CODE END StartGoStraight */
}

/* USER CODE BEGIN Header_StartRightTurn */
/**
* @brief Function implementing the RightTurn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRightTurn */
void StartRightTurn(void const * argument)
{
  /* USER CODE BEGIN StartRightTurn */
  /* Infinite loop */
  for(;;)
  {

	  if(Avrg_Ctr==1)
	  osThreadSetPriority(ADCHandle,osPriorityBelowNormal);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	  if(TurnAround==1)
	  {

	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
	  }
	  else
	  {
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1400);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	  }
	  if(SUMDe==0)
		  A=1;
	  if(A==1&&(SUMDe>3))
	  {
		  TurnAround=0;
		 osThreadSetPriority(GoStraightHandle,osPriorityLow);
		 osThreadSetPriority(RightTurnHandle,osPriorityIdle);
	  }






  }
  /* USER CODE END StartRightTurn */
}

/* USER CODE BEGIN Header_StartLefTurn */
/**
* @brief Function implementing the LeftTurn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLefTurn */
void StartLefTurn(void const * argument)
{
  /* USER CODE BEGIN StartLefTurn */
  /* Infinite loop */
  for(;;)
  {
	  if(Avrg_Ctr==1)
	  osThreadSetPriority(ADCHandle,osPriorityBelowNormal);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,700);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,700);
	  if(SUMDe==0)
		  A=1;
	  if(A==1&&(SUMDe!=0))
	  {
		  A=0;
		 osThreadSetPriority(GoStraightHandle,osPriorityLow);
		 osThreadSetPriority(LeftTurnHandle,osPriorityIdle);
	  }
  }
  /* USER CODE END StartLefTurn */
}

/* USER CODE BEGIN Header_StartGoBack */
/**
* @brief Function implementing the GoBack thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGoBack */
void StartGoBack(void const * argument)
{
  /* USER CODE BEGIN StartGoBack */
  /* Infinite loop */
  for(;;)
  {

	  if(Avrg_Ctr==1)
	 osThreadSetPriority(ADCHandle,osPriorityBelowNormal);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_14, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);
	  if(Error>0)
	  {
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	  }
	  else
	  {
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
	  }
	  if(SUMDe>3)
	  {
	   I_Ctrl=0;
	  osThreadSetPriority(GoBackHandle,osPriorityIdle);
	  osThreadSetPriority(GoStraightHandle,osPriorityLow);
	  }

  }
  /* USER CODE END StartGoBack */
}

/* USER CODE BEGIN Header_StartADC */
/**
* @brief Function implementing the ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC */
void StartADC(void const * argument)
{
  /* USER CODE BEGIN StartADC */
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Stop(&hadc1);
	hdma_adc1.Init.Mode = DMA_CIRCULAR;
	HAL_DMA_Init(&hdma_adc1);
	HAL_ADC_Start_DMA(&hadc1, ADCsensor,10);
	osThreadSetPriority(ADCHandle,osPriorityIdle);
  }
  /* USER CODE END StartADC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
