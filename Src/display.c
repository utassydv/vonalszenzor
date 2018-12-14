/*
 * display.c
 *
 *  Created on: 11 Dec 2018
 *      Author: utassyd
 */
#include "stm32f4xx_hal.h"
#include "display.h"


typedef union{
	uint8_t rawdata[4];
	uint32_t word;
} spiDataUnion_t;


extern float dist;
extern SPI_HandleTypeDef hspi2;

spiDataUnion_t vjLED;
spiDataUnion_t teszt;

void displayLinePos(uint32_t tav)									//tav alapjan visszajelzes (0-26425.6) 26425.6/32=8.258
{


	uint8_t ledpos;
	vjLED.word=0;

	ledpos = 30-(uint32_t)(tav / (dist*100.0f));

	vjLED.word = 3 << ledpos;


	//Adatkldťs
	HAL_SPI_Transmit(&hspi2, vjLED.rawdata, 4, HAL_MAX_DELAY);
	//LE (PB12 33-NSS) fel le
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

}

/*
//Binaris visszajelzes:
	//void BvjLED(adcMeasures);
	vjLED.word=0;
	for(uint8_t v=0; v<32; v++)
	{
		if(adcMeasures[v] > 800)
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
*/
