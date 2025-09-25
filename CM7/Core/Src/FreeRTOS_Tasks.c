/*
 * FreeRTOS_Tasks.c
 *
 *  Created on: Sep 23, 2025
 *      Author: q
 */




#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_Tasks.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_uart.h"

volatile TaskHandle_t CurrentTask = NULL;

extern UART_HandleTypeDef huart3;

extern TaskHandle_t TaskLED_Green;
extern TaskHandle_t TaskLED_Yellow;
extern SemaphoreHandle_t xMyMutex;

void TaskLedGreen_Handle(void *Pram)
{

//	BaseType_t xResult;

	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOB, LED_GRE_Pin);
		SEGGER_SYSVIEW_Print("Toggle Green LED");
		if(xSemaphoreTake(xMyMutex, portMAX_DELAY))
		{
			HAL_UART_Transmit(&huart3, (uint8_t *)"Toggle Green LED\r\n", strlen("Toggle Green LED\r\n"), HAL_MAX_DELAY);
			xSemaphoreGive(xMyMutex);
		}
		taskYIELD();
//		xResult = xTaskNotifyWait(0x00, 0x00, NULL, pdMS_TO_TICKS(500));
//		if(xResult == pdTRUE)
//		{
//			if(CurrentTask == TaskLED_Yellow)
//			{
//				vTaskSuspendAll();
//				vTaskPrioritySet(TaskLED_Green, 3);
//				vTaskPrioritySet(TaskLED_Yellow, 4);
//				xTaskResumeAll();
//			}
//
//		}

//		vTaskDelay(pdMS_TO_TICKS(500));
	}

}

void TaskLedYellow_Handle(void *Pram)
{
//	BaseType_t xResult;

	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOE, LED_YEL_Pin);
		SEGGER_SYSVIEW_Print("Toggle Yellow LED");
//		xResult = xTaskNotifyWait(0x00, 0x00, NULL, pdMS_TO_TICKS(500));
		if(xSemaphoreTake(xMyMutex, portMAX_DELAY))
		{
			HAL_UART_Transmit(&huart3, (uint8_t *)"Toggle Yellow LED\r\n", strlen("Toggle Yellow LED\r\n"), HAL_MAX_DELAY);
			xSemaphoreGive(xMyMutex);
		}
		taskYIELD();
//		if(xResult == pdTRUE)
//		{
//			if(CurrentTask == TaskLED_Green)
//			{
//				vTaskSuspendAll();
//				vTaskPrioritySet(TaskLED_Yellow, 3);
//				vTaskPrioritySet(TaskLED_Green, 4);
//				xTaskResumeAll();
//			}
//
//		}
//		vTaskDelay(pdMS_TO_TICKS(500));
	}

}


