/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "MAX7219.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t FINISH_FL=0,TIMER_START=1,ADC_FINISH=1,RETRY=10;
volatile uint32_t ADC_VALUE=0;
 uint32_t CAL_VALUE,DISPLAY_VALUE,temp[100];
														//abcdefg

const uint8_t	seven_seg_digits[] =
{
  0x7E,//0   0
	0x30,//1   
	0x6D,//2
	0x79,//3
	0x33,//4
	0x5B,//5
	0x5F,//6
	0x70,//7
	0x7F,//8
	0x7B,//9   9
	0x77,//a   10
	0x1F,//b   11
	0x4E,//c   12
	0x3D,//d   13
	0x4F,//e   14
	0x47,//f   15
	0xE, //L   16
	0x4E,//[   17
	0x78  //]  18
	
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint16_t ii;
uint8_t RUN=1;
uint32_t GET_OPACITY(void);
void DISP_OPACITY(uint32_t DISPLAY_VALUE);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
		
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
init_7219();

ADC_Enable(&hadc1);
HAL_ADC_Start_IT(&hadc1);
HAL_ADC_GetValue(&hadc1);



for(ii=1;ii<8;ii++)
{		
	HAL_Delay(300);
	SendData_7219(1,128>>ii);
	SendData_7219(2,128>>ii);
	SendData_7219(3,128>>ii);
	SendData_7219(4,128>>ii);									
}		

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {				

		
		if(HAL_GPIO_ReadPin(BUTTON_3_GPIO_Port, BUTTON_3_Pin)==0) // Калібрування пристрою
{
	SendData_7219(1,0x0);
	SendData_7219(2,seven_seg_digits[16]);
	SendData_7219(3,seven_seg_digits[10]);
	SendData_7219(4,seven_seg_digits[12]);

	
HAL_Delay(100);

CAL_VALUE=GET_OPACITY();
	
	DISP_OPACITY(GET_OPACITY());
	
	
};
if(HAL_GPIO_ReadPin(BUTTON_2_GPIO_Port, BUTTON_2_Pin)==0) // Вимірювання
{
	RUN=1;
	

					while(RUN)
					{
					HAL_Delay(100);
					DISP_OPACITY(GET_OPACITY());		
						
							if(HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin)==0) // ЗАПИС
							{
							RUN=0;
							HAL_Delay(100);
							}
					}
					
}


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
	
		
		
		
//SendData_7219(1,6);
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 100;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 13700;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_TX_Pin|BUZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_TX_Pin */
  GPIO_InitStruct.Pin = LED_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZ_Pin */
  GPIO_InitStruct.Pin = BUZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_3_Pin BUTTON_2_Pin BUTTON_1_Pin */
  GPIO_InitStruct.Pin = BUTTON_3_Pin|BUTTON_2_Pin|BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t GET_OPACITY(void)
	{
		
	
		uint32_t ADC_SUM=0;
		uint8_t CNT_RETRY=0,ADC_FL=0;
		
			ADC_SUM=0;
	HAL_TIM_Base_Start_IT(&htim2); 				//запуск таймера
		for(CNT_RETRY=0;CNT_RETRY<RETRY;CNT_RETRY++)
		{
			ADC_FL=0;
			TIM2->CNT=0;												//обнуляю таймер
			HAL_GPIO_WritePin(LED_TX_GPIO_Port,LED_TX_Pin,GPIO_PIN_SET); //запалюю світлодіод
			
			while(TIM2->CNT!=1500)							//очікую 1500мкс
				{
					if((TIM2->CNT==500)&(ADC_FL==0)) 	//через 500мкс після старту читаю АЦП в массив
							{
								ADC_FINISH=0;
								HAL_ADC_Start_IT(&hadc1);
								while(!ADC_FINISH){}
								
							  temp[CNT_RETRY]=HAL_ADC_GetValue(&hadc1); //сумую значення АЦП
								ADC_FL=1;
							}
				}
				HAL_GPIO_WritePin(LED_TX_GPIO_Port,LED_TX_Pin,GPIO_PIN_RESET); //тушу світлодіод по завершенню 1500 мкс
				while(!TIMER_START);											//очікування переповнення таймера
				TIMER_START=0;												//зкидаю флаг переповнення таймера
			
			}
		HAL_TIM_Base_Stop_IT(&htim2); 				//зупинка таймера
			
			for(CNT_RETRY=0;CNT_RETRY<RETRY;CNT_RETRY++)
		{
			ADC_SUM=ADC_SUM+temp[CNT_RETRY];
		}
			DISPLAY_VALUE=ADC_SUM/RETRY;					//рахую середнє значення
		return DISPLAY_VALUE;										//повертаю результуюче значення
	}	
	

void DISP_OPACITY(uint32_t DISPLAY_VALUE)
{
	uint8_t dig[5];
	uint8_t i;
	uint32_t out_value,num;
	out_value=((1000*DISPLAY_VALUE)/CAL_VALUE);
	//out_value=(100.0/CAL_VALUE)*DISPLAY_VALUE;
	
	
//	out_value=DISPLAY_VALUE;
	//char CHAR_VALUE[10];
	//sprintf(CHAR_VALUE,"%5.1f",out_value);
	//Dec7219(out_value,4);
	
num=out_value;
	
	for(i=1;i<4+1;i++)
	{	
	dig[i]=(i,seven_seg_digits[num%10]);
	num=(num/10); 
	}
	
	SendData_7219(1,dig[1]);
	SendData_7219(2,dig[2]+128);
	SendData_7219(3,dig[3]);
	SendData_7219(4,dig[4]);
}	


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
