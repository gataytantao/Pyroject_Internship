//DHT_22.h
#include "stm32f4xx_hal.h"
//Pin read/Write Data from DHT11 Sensor
#define DHT11_PORT GPIOA 
#define DHT11_PIN GPIO_PIN_3

//Add in main() (Note: before while(1))
void init_DHT22(void);
void delayus(uint16_t time);					//Delay in microsec.
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
//Read 1 byte (in 5 bytes transmission)
uint8_t DHT22_Read (void);
//Check DHT connection and data valid???
uint8_t DHT22_Check_Response(uint8_t d[5]);
