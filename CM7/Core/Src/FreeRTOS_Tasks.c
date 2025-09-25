/*
 * FreeRTOS_Tasks.c
 *
 *  Created on: Sep 23, 2025
 *      Author: q
 */




#include <stdio.h>
#include <string.h>
#include "FreeRTOS_Tasks.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"





void TaskLedGreen_Handle(void *Pram)
{
	uint8_t array[20];
	uint8_t lenght = 0;
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOB, LED_GRE_Pin);
		sprintf((char*)array,"Toggle Green LED\n");
		lenght = (uint8_t)strlen((char*)array);
		SEGGER_SYSVIEW_Print("Toggle Green LED");
		vTaskDelay(pdMS_TO_TICKS(500));
	}

}
void TaskLedRed_Handle(void *Pram)
{
	uint8_t array[20];
	uint8_t lenght = 0;
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);
		sprintf((char*)array,"Toggle Red LED\n");
		lenght = (uint8_t)strlen((char*)array);
		SEGGER_SYSVIEW_Print("Toggle Red LED");
		vTaskDelay(pdMS_TO_TICKS(500));
	}

}
void TaskLedYellow_Handle(void *Pram)
{
	uint8_t array[20];
	uint8_t lenght = 0;
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOE, LED_YEL_Pin);
		sprintf((char*)array,"Toggle Yellow LED\n");
		lenght = (uint8_t)strlen((char*)array);
		SEGGER_SYSVIEW_Print("Toggle Yellow LED");
		vTaskDelay(pdMS_TO_TICKS(500));
	}

}
