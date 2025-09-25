/*
 * FreeRTOS_Tasks.h
 *
 *  Created on: Sep 23, 2025
 *      Author: q
 */

#ifndef INC_FREERTOS_TASKS_H_
#define INC_FREERTOS_TASKS_H_

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

void TaskLedGreen_Handle(void *Pram);

void TaskLedRed_Handle(void *Pram);

void TaskLedYellow_Handle(void *Pram);




#endif /* INC_FREERTOS_TASKS_H_ */
