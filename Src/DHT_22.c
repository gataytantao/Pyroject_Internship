//DHT_22.c
#include "DHT_22.h"
extern TIM_HandleTypeDef htim6;

void init_DHT22(void)
{
	HAL_TIM_Base_Start(&htim6);
}

void delayus(uint16_t time)							//Code 1: Timer 6
{
	//change your code here for the delay in microseconds 
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim6))<time);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DHT22_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
		delayus(30);					//Same as DHT11 and DHT22
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
		{
			while (HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN));
			i |= (1<<(7-j));
		}
		else i &= ~(1<<(7-j));
	}
	return i;
}

uint8_t DHT22_Check_Response(uint8_t d[])
{
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
	delayus(1000);							//For DHT22, DHT11 takes 20000us
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);
	delayus(40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
		d[0] = DHT22_Read();
		d[1] = DHT22_Read();
		d[2] = DHT22_Read();
		d[3] = DHT22_Read();
		d[4] = DHT22_Read();
		Set_Pin_Output(DHT11_PORT, DHT11_PIN);
		HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
		//if(d[4] == d[3]+d[2]+d[1]+d[0] && d[4] >= 256)
			return 1;
		//else return 0;			//Check sum ko on => sum > 256 is not ok
	}
	else
	{
		Set_Pin_Output(DHT11_PORT, DHT11_PIN);
		HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
		return 0;
	}
}
