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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd_16x2.h"
#include "string.h"
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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

osThreadId Tast01Handle;
osThreadId Task02Handle;
osThreadId Task03Handle;
osThreadId Task04Handle;
/* USER CODE BEGIN PV */


uint16_t var[3];
uint16_t gasVariable=0,fireVariable=0,tempVariable=0;
//uint8_t gas=0, fire=0, temp=0;
char a[20];
uint8_t buff[10];
volatile uint8_t send,resend;
static volatile uint8_t tempRange=80,fireRange=80,gasRange=80;
static volatile uint8_t menu,chose;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void DisplayLCD(void const * argument);
void Warning(void const * argument);
void TogglePin(void const * argument);
void ReadADC(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





							/*HAM CHONG DOI NUT NHAN */

uint8_t debounce(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 0)
	    {
	    	HAL_Delay(80);
	        if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 0)
	        {
	            return 1;
	        }
	    }
	    return 0;
}







                                  /* HAM DE HIEN THI LEN LCD */
void HienThiCacDonVi (void)
{

	uint8_t degreeC[8] = { 0b00110, 0b00110 };    /*cai nay de tao ra dau do*/
	/*Nhan nut menu*/
	if (debounce(GPIOB, GPIO_PIN_0)==1)
	{
		Lcd_clear_display();
		menu=menu+1;
	}


	if(menu==0)
	{
	Lcd_gotoxy(0, 0);
	sprintf(a, "%d  ", tempVariable);
	Lcd_write_string(a);
	Lcd_gotoxy(11, 0);
	sprintf(a, "%d  ", gasVariable);
	Lcd_write_string(a);
	Lcd_gotoxy(11, 1);
	sprintf(a, "%d  ", fireVariable);
	Lcd_write_string(a);
	Lcd_create_custom_char(0, degreeC);
	Lcd_write_custom_char(3, 0, 0);
	Lcd_gotoxy(4, 0);
	Lcd_write_string("C");
	Lcd_gotoxy(8, 0);
	Lcd_write_string("Ga:");
	Lcd_gotoxy(15, 0);
	Lcd_write_string("%");
	Lcd_gotoxy(8, 1);
	Lcd_write_string("Fi:");
	Lcd_gotoxy(15, 1);
	Lcd_write_string("%");
	}


		if(menu==1)
		{
			Lcd_gotoxy(0, 0);
			Lcd_write_string("Temp range:");
			Lcd_create_custom_char(0, degreeC);
			Lcd_write_custom_char(3, 1, 0);
			Lcd_gotoxy(4, 1);
			Lcd_write_string("C");
			Lcd_gotoxy(0, 1);
			Lcd_write_int(tempRange);
			if (debounce(GPIOB, GPIO_PIN_1)==1)
			{
				tempRange++;
				Lcd_clear_xy(0,1);
				Lcd_clear_xy(1,1);
				Lcd_clear_xy(2,1);
				if(tempRange >150)
				{
					tempRange=20;
				}
			}
			if (debounce(GPIOB, GPIO_PIN_3)==1)
			{
				tempRange--;
				Lcd_clear_xy(0,1);
				Lcd_clear_xy(1,1);
				Lcd_clear_xy(2,1);
				if(tempRange<20)
				{
					tempRange=150;
				}
			}
		}


		if(menu==2)
			{
				Lcd_gotoxy(0, 0);
				Lcd_write_string("Gas range:");
				Lcd_gotoxy(0, 1);
				Lcd_write_int(gasRange);
				Lcd_gotoxy(4, 1);
				Lcd_write_string("%");
				if (debounce(GPIOB, GPIO_PIN_1)==1)
				{
					gasRange++;
					Lcd_clear_xy(0,1);
					Lcd_clear_xy(1,1);
					Lcd_clear_xy(2,1);
					if(gasRange>100)
					{
						gasRange=0;
					}
				}
				if (debounce(GPIOB, GPIO_PIN_3)==1)
				{
					gasRange--;
					Lcd_clear_xy(0,1);
					Lcd_clear_xy(1,1);
					Lcd_clear_xy(2,1);
					if(gasRange<0)
					{
						gasRange=100;
					}
				}
			}



		if(menu==3)
			{
				Lcd_gotoxy(0, 0);
				Lcd_write_string("Fire range:");
				Lcd_gotoxy(0, 1);
				Lcd_write_int(fireRange);
				Lcd_gotoxy(4, 1);
				Lcd_write_string("%");
				if (debounce(GPIOB, GPIO_PIN_1)==1)
				{
					fireRange++;
					Lcd_clear_xy(0,1);
					Lcd_clear_xy(1,1);
					Lcd_clear_xy(2,1);
					if(fireRange>100)
					{
						fireRange=0;
					}
				}
				if (debounce(GPIOB, GPIO_PIN_3)==1)
				{
					fireRange--;
					Lcd_clear_xy(0,1);
					Lcd_clear_xy(1,1);
					Lcd_clear_xy(2,1);
					if(fireRange<0)
					{
						fireRange=100;
					}
				}
			}




		if(menu>3)
		{
			menu=0;
		}
	}







										/*   HAM GUI TIN HIEU SMS    */
void SendWarning()
{
	const char* warningMsg1 = "AT+CMGS=\"0765214176\"\r\n";
	HAL_UART_Transmit(&huart1,(uint8_t*)warningMsg1,strlen(warningMsg1),HAL_MAX_DELAY);
	HAL_Delay(5000);
	const char* warningMsg2 = "WARNING VERY HIGH PRESS '1' TO SHUT DOWN\x1A\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)warningMsg2, strlen(warningMsg2),HAL_MAX_DELAY);
	HAL_Delay(5000);
	const char* warningMsg3 = "AT+CMGD=1,4\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)warningMsg3, strlen(warningMsg3),HAL_MAX_DELAY);
	HAL_Delay(10000);
}






                                        /*    HAM CANH BAO MUC DO   */

int CanhBao (uint16_t a /*temp*/, uint16_t b /*gas*/, uint16_t c /*fire*/ )
{

	if (/*a >= tempRange ||*/ b >= gasRange || c >= fireRange)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


                           /*       NGAT NGOAI XAY RA KHI CO BUFFER NHAN VE     */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
		  if (buff[1] == '1' && send == 1)
	          {
	              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	              memset(buff, 0, sizeof(buff));
	              const char* warningMsg4 = "SYSTEM NOW OFF\n";
	              HAL_UART_Transmit(&huart1,(uint8_t *)warningMsg4,strlen(warningMsg4),HAL_MAX_DELAY);
	              resend = 1;
	          }
	          HAL_UART_Receive_IT(&huart1, buff, sizeof(buff));
  }
}



void BuzzerSound()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,1);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,0);
	HAL_Delay(100);
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
  Lcd_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  memset(buff, 0, sizeof(buff));
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) var, 3);
//  HAL_UART_Receive_IT(&huart1, buff, 1);
  Lcd_clear_display();
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
  /* definition and creation of Tast01 */
  osThreadDef(Tast01, DisplayLCD, osPriorityNormal, 0, 128);
  Tast01Handle = osThreadCreate(osThread(Tast01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, Warning, osPriorityNormal, 0, 128);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* definition and creation of Task03 */
  osThreadDef(Task03, TogglePin, osPriorityNormal, 0, 128);
  Task03Handle = osThreadCreate(osThread(Task03), NULL);

  /* definition and creation of Task04 */
  osThreadDef(Task04, ReadADC, osPriorityNormal, 0, 128);
  Task04Handle = osThreadCreate(osThread(Task04), NULL);

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
		/*    GAN GIA TRI    */

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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_DisplayLCD */
/**
  * @brief  Function implementing the Tast01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DisplayLCD */
void DisplayLCD(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HienThiCacDonVi();
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Warning */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Warning */
void Warning(void const * argument)
{
  /* USER CODE BEGIN Warning */
	HAL_UART_Receive_IT(&huart1, buff, 1);
  /* Infinite loop */
  for(;;)
  {
	if(CanhBao(tempVariable,gasVariable,fireVariable) == 1  && resend == 0)
	{
			SendWarning();
	}
    osDelay(1);
  }
  /* USER CODE END Warning */
}

/* USER CODE BEGIN Header_TogglePin */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TogglePin */
void TogglePin(void const * argument)
{
  /* USER CODE BEGIN TogglePin */
  /* Infinite loop */
  for(;;)
  {
	if(CanhBao(tempVariable,gasVariable,fireVariable) == 1)
	{
		if(resend == 1)
		{
			send=0;
		}
		else
		{
			BuzzerSound();
			send=1;
		}
	}
	else if(CanhBao(tempVariable,gasVariable,fireVariable) == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,0);
		send=0;
	}
    osDelay(1);
  }
  /* USER CODE END TogglePin */
}

/* USER CODE BEGIN Header_ReadADC */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadADC */
void ReadADC(void const * argument)
{
  /* USER CODE BEGIN ReadADC */
  /* Infinite loop */
  for(;;)
  {
	tempVariable=((var[0]*500)/4095);
	gasVariable=((var[1] * 100) / 4095);
	fireVariable=(((4095 - var[2]) * 100) / 4095);
    osDelay(1);
  }
  /* USER CODE END ReadADC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
	__disable_irq();
	while (1) {
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
