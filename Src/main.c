
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "stdio.h"
//#include "stdlib.h"
#include <stdbool.h>
#include <string.h>
#include "display.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t i,j,k;
extern uint32_t myTicks;
char TxData[16]; // "2,150000'\0'"
uint8_t count;    //vonal db szĹ m
uint16_t pos;
float yhszum=0;
float hszum=0;
uint32_t tav= 12799;		//kozep
uint32_t tav2 = 12799;
uint32_t regitav= 12799;	//kozep

uint8_t counter=0;
uint8_t szinkr=0;
uint8_t kuldcpl=1;
uint8_t datacpl=0;
const float dist=8.258;

uint8_t felfuto;
uint8_t lefuto;

uint8_t countfel=0;
uint8_t countle=0;



uint32_t channelLUT[16][2] = {
	{ADC_CHANNEL_0,	 ADC_CHANNEL_2},
	{ADC_CHANNEL_11, ADC_CHANNEL_13},
	{ADC_CHANNEL_0,	 ADC_CHANNEL_2},
	{ADC_CHANNEL_11, ADC_CHANNEL_13},
	{ADC_CHANNEL_0,	 ADC_CHANNEL_2},
	{ADC_CHANNEL_11, ADC_CHANNEL_13},
	{ADC_CHANNEL_0,	 ADC_CHANNEL_2},
	{ADC_CHANNEL_11, ADC_CHANNEL_13},
	{ADC_CHANNEL_1,	 ADC_CHANNEL_3},
	{ADC_CHANNEL_10, ADC_CHANNEL_12},
	{ADC_CHANNEL_1,	 ADC_CHANNEL_3},
	{ADC_CHANNEL_10, ADC_CHANNEL_12},
	{ADC_CHANNEL_1,	 ADC_CHANNEL_3},
	{ADC_CHANNEL_10, ADC_CHANNEL_12},
	{ADC_CHANNEL_1,	 ADC_CHANNEL_3},
	{ADC_CHANNEL_10, ADC_CHANNEL_12}
};

uint16_t iLEDLUT[9] = 	{
							0b0000000100000001,
							0b0000001000000010,
							0b0000010000000100,
							0b0000100000001000,
							0b0001000000010000,
							0b0010000000100000,
							0b0100000001000000,
							0b1000000010000000,
							0b0000000000000000
						};

uint16_t measureLUT[32] = {  0,  8, 16, 24,
							1,  9, 17, 25,
							2, 10, 18, 26,
							3, 11, 19, 27,
							4, 12, 20, 28,
							5, 13, 21, 29,
							6, 14, 22, 30,
							7, 15, 23, 31
							};

//MUX vezĹĄrlĹĄsĹĄre
uint8_t muxLUT[16] = { 	3, 0, // 1,  9, 17, 25
						0, 2, // 2, 10, 18, 26
						1, 3, // 3, 11, 19, 27
						2, 1, // 4, 12, 20, 28
						0, 3, // 5, 13, 21, 29
						2, 0, // 6, 14, 22, 30
						3, 1, // 7, 15, 23, 31
						1, 2 // 8, 16, 24, 32
						};

uint16_t adcMeasures[32];
uint16_t measureIndex;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void setLEDs(uint8_t index);
void measureADC(uint8_t index);
void measure(void);
void Delay_us(uint16_t us);
void setMux(uint8_t index);
uint16_t vonaltav_calc (uint16_t* ertekek, uint8_t szam);
uint16_t vonalszam_calc (uint16_t* ertekek);
void vonalszenzordebug(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);

//ALAPHELYZETBE Ĺ�LLĂ•TĹ�S
  //VISSZAJELZâ€™ LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //LE lehÄ·z vj
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);//oe
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);//oe
  //SZENZOR LED
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //LE lehÄ·z sz
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //OE
HAL_Delay(10);

 while(1)
 {

	if (szinkr)
	{
		measure();										//meres
//		adcMeasures[0] = 0;								//rossz erteket ad ez a szenzor
//		adcMeasures[1] = 0;

		count 	= vonalszam_calc (adcMeasures);			//vonalszam meghatarozas
		tav		= vonaltav_calc (adcMeasures, count);	//vonalpozicio meghatarozas

		displayLinePos(tav);							//tav alapjan visszajelzes (0-26425.6) 26425.6/32=8.258

		szinkr = 0;										//meres utemezeshez
		datacpl = 1;									//kuldes kesz visszajelzes
	}

//Kuldendo adatcsomag letrehozasa
	snprintf(TxData, 16, "%u,%lu,%lu\0", count, tav, tav2); //"2,150000'\0'"


//KULDES
	if (datacpl && kuldcpl)
	{
		kuldcpl=0;
		HAL_UART_Transmit_IT(&huart5, (uint8_t *)TxData, (strlen(TxData)+1)); //melyik, mit, mennyi, mennyi ido
		//vonalszenzordebug();
		datacpl=0;
	}

 }


  /* USER CODE END 2 */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 90;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void setMux(uint8_t index)
{
	uint8_t muxIndex = muxLUT[index];
	switch(muxIndex){
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		break;
	default:
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
	  counter++;

	  //10ms-os idozites
	  if(counter==5)
	  {
		  szinkr=1;
		  counter=0;
	  }


  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  kuldcpl=1;
}

void setLEDs(uint8_t index)
{
	uint16_t data = iLEDLUT[index];
	HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //LE fel
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //LE le
}

uint8_t measureCompleted = 0;
void measureADC(uint8_t index)
{
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = channelLUT[index][0];
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}


	sConfig.Channel = channelLUT[index][1];
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	measureCompleted = 0;
	uint32_t adcData[2];
	if(HAL_ADC_Start_DMA(&hadc1, adcData, 2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	while(measureCompleted == 0);
	adcMeasures[measureLUT[2*index]] = (uint16_t)adcData[0];
	adcMeasures[measureLUT[2*index+1]] = (uint16_t)adcData[1];
	HAL_ADC_Stop_DMA(&hadc1);
}

void measure(void)
{
	for(i=0; i<8; i++)
	{
		setLEDs(i);
		for(j = 0; j < 2; j++)
		{
			setMux(i*2+j);
			measureADC(i*2+j);
		}
	}
	setLEDs(9);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	measureCompleted = 1;
}

void Delay_us(uint16_t us)
{
	uint32_t vege=myTicks + us;
	while(myTicks < vege);
}

/*
void BvjLED(uint16_t* measures) //Binaris visszajelzes:
{
		vjLED.word=0;
		for(int v=0; v<32; v++)
		{
			if(measures[v] > 800)
			{
				vjLED.word =vjLED.word|1;
			}
			if(v<31) vjLED.word = vjLED.word << 1;
		}

		//teszt.word=0b00000000000000000000000000000000;

		//Adatk�?ż�??ldĹĄs
		HAL_SPI_Transmit(&hspi2, vjLED.rawdata, 4, HAL_MAX_DELAY);
		//LE (PB12 33-NSS) fel le
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}
*/

//vonalszam kalkulacio
uint16_t vonalszam_calc (uint16_t* ertekek)
{
	countfel = 0;
	countle = 0;

	uint16_t meresek[32];
	uint16_t hatar=600;

	for(int i=0; i<32; i++)
	{
		meresek[i]=ertekek[i];
	}

//	int max=0, min=meresek[0];
//
//	for(int j=0; j<32; j++)
//	{
//		if(meresek[j] > max) max=meresek[j];
//		if(meresek[j] < min) min=meresek[j];
//	}
//	min=min+((max-min)/4);
//	for(int j=0; j<32; j++)
//		{
//			if(meresek[j]<min) meresek[j]=0;
//	}
	uint8_t flagle = 0;
	for(int i=0, j=1 ; j<32 ; i++, j++ )								//egyik kisebb-másik nagyobb mint "hatar"
	{
			if(  (meresek[i] < hatar) && (meresek[j] > hatar) )
			{
				countfel++;
				felfuto = i;
			}
			if( (meresek[j] < hatar) && (meresek[i] > hatar) )
			{
				countle++;
				if(flagle == 0)
				{
					lefuto = j;
					flagle = 1;
				}
			}
	}

//	for(int i=0, j=1 ; j<32 ; i++, j++ )								//"hatar" különbség van a kettő között
//	{
//			if( (meresek[i] +  hatar) < meresek[j] )
//			{
//				countfel++;
//			}
//			if( (meresek[i] -  hatar) > meresek[j] )
//			{
//				countle++;
//			}
//	}

	if (countfel < countle)
	{
		countfel=countle;
	}
	return countfel;
}



uint16_t vonaltav_calc (uint16_t* ertekek, uint8_t szam)
{

	yhszum=0;
	hszum=0;
	static float sum=0;
	static uint8_t init = 0;

	if(init == 0) {						//csak az elejen szamol atlagot
		for(int i=0; i<32; i++)
		{
			sum = sum + ertekek[i];
		}

		sum=sum/32.0f;
		init = 1;
	}

	for(int i=0; i<32; i++)				//atlagnal kisebbet nullaz, nagyobból kivonja az atlagot
	{
		if(ertekek[i]<sum)
		{
			ertekek[i]=0;
		}
		else
		{
			ertekek[i]=ertekek[i]-sum;
		}
	}

	if( szam == 2 )
	{
		uint8_t kozep = (lefuto + felfuto) / 2;

		//ELSO VONAL
		for(int i=0 ; i< kozep ; i++)
		{
			yhszum=yhszum+ertekek[i]*i*dist;
			hszum=hszum+ertekek[i];
		}
		tav= (yhszum/hszum)*100; //0-25599

		//MASODIK VONAL
		yhszum=0;
		hszum=0;
		for(int i=kozep ; i< 32 ; i++)
		{
			yhszum=yhszum+ertekek[i]*i*dist;
			hszum=hszum+ertekek[i];
		}
		tav2= (yhszum/hszum)*100; //0-25599


	}
	else
	{
		for(int i=0 ; i<32 ; i++)
		{
			yhszum=yhszum+ertekek[i]*i*dist;
			hszum=hszum+ertekek[i];
		}

		if (szam==0)
		{
			tav= regitav;
			tav2 = 0;
		}
		else
		{
			tav= (yhszum/hszum)*100; //0-25599
			tav2 = 0;
			regitav=tav;
		}
	}



	return tav;
}

void vonalszenzordebug(void)
{
	uint8_t debugdata[32];

	for(int i=0; i<32; i++)
	{
		if (adcMeasures[i] > 2500)
		{
			debugdata[i] = 250;
		}
		else
		{
			debugdata[i] = adcMeasures[i]/10;
		}
	}


	char TxDatak[200];
	snprintf(TxDatak,200,"%u,",debugdata[0] );

	for(int i=1; i<32; i++)
	{
		char TxBuf[5];
		snprintf(TxBuf,5,"%u,", debugdata[i]);
		strcat(TxDatak, TxBuf);
	}
	strcat(TxDatak, "\n");

	HAL_UART_Transmit(&huart5, (uint8_t *)TxDatak, strlen(TxDatak)+1 , HAL_MAX_DELAY); //melyik, mit, mennyi, mennyi ido
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
