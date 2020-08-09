#ifndef _MY_PROTOTYPES_H_
#define _MY_PROTOTYPES_H_

#include "stdio.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

void rtos_delay(uint32_t delayInMS);

void printMsg(char *msg);
void setLED(void);
void resetLED(void);
void toggleLED(void);
void vSetupHardware(void);

extern TaskHandle_t xButtonHandle;
extern TaskHandle_t xLEDHandle;
void vTaskButton(void *params);
void vTaskLED(void *params);

#endif
