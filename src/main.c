/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stdio.h"
//#include "stdint.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

//for arm semihosting for printf
//changed in debug startup, linker and in main
//check udemy understanding arm semihosting, lesson 40
static void prvSetupHardware(void);
extern void initialise_monitor_handles(void);

void vTask1Function(void *params);
void vTask2Function(void *params);

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;

int main(void)
{
//to use semihosting add this macro to preprocessor symbols
//project->properties->c/c++Build->preprocessor->add->USE_SEMIHOSTING
#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
#endif
	RCC_DeInit();//reset RCC to default config of HSI clock of 16 Mhz
	SystemCoreClockUpdate();//update to default clock

	//Task Prototype
	//xTaskCreate(TaskFunction_t pxTaskCode,
	//const char *const pcName,
	//const configSTACK_DEPTH_TYPE usStackDepth,
	//void * const pvParameters,
	//UBaseType_t uxPriority,
	//TaskHandle_t * const pxCreatedTask )

	//create tasks
	xTaskCreate(vTask1Function,"Task1",configMINIMAL_STACK_SIZE,NULL,2,&xTaskHandle1);
	xTaskCreate(vTask2Function,"Task2",configMINIMAL_STACK_SIZE,NULL,2,&xTaskHandle2);

	//start scheduler
	vTaskStartScheduler();
	for(;;);
}


void vTask1Function(void *params)
{
	for(;;)
	{
#ifdef USE_SEMIHOSTING
		printf("task 1 printing\n");
#endif
	}
}

void vTask2Function(void *params)
{
	for(;;)
	{
#ifdef USE_SEMIHOSTING
		printf("task 2 printing\n");
#endif
	}
}
static void prvSetupHardware(void)
{

}
