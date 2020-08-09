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
#include <string.h>


static void prvSetupHardware(void);
static void prvSetupUART(void);
void printMsg(char *msg);
//for arm semihosting for printf
//changed in debug startup, linker and in main
//check udemy understanding arm semihosting, lesson 40

#ifdef USE_SEMIHOSTING
extern void initialise_monitor_handles(void);
#endif
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
	prvSetupHardware();


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
		printMsg("task 1 printing\n\r");
#ifdef USE_SEMIHOSTING
		printf("task 1 printing\n");
#endif
	}
}

void vTask2Function(void *params)
{
	for(;;)
	{
		printMsg("task 2 printing\n\r");
#ifdef USE_SEMIHOSTING
		printf("task 2 printing\n");
#endif
	}
}
static void prvSetupUART(void)
{
	GPIO_InitTypeDef  gpioConfig;
	USART_InitTypeDef usartConfig;
	//enable USART6 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	//enable GPIOC peripheral bus
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	//USART pin configuration
	//set PC6 as TX and PC7 as RX
	memset(&gpioConfig,0,sizeof(gpioConfig));
	gpioConfig.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//set both 6 and 7 as GPIO
	gpioConfig.GPIO_Mode = GPIO_Mode_AF;	//set alternate function mode
	gpioConfig.GPIO_PuPd = GPIO_PuPd_UP;	//enable pull up on both pins
	GPIO_Init(GPIOC, &gpioConfig);			//set the pin config
	//set GPIO alternate function
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);//enable alternate functions on usart6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);

	//init USART
	memset(&usartConfig,0,sizeof(usartConfig));
	usartConfig.USART_BaudRate = 9600;
	usartConfig.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartConfig.USART_Parity = USART_Parity_No;
	usartConfig.USART_StopBits = USART_StopBits_1;
	usartConfig.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART6,&usartConfig);

	//enable USART peripheral
	USART_Cmd(USART6,ENABLE);
}

//This function will do all HW related settings
static void prvSetupHardware(void)
{
	prvSetupUART();
}
void printMsg(char *msg)
{
	for(uint32_t i=0;i<strlen(msg);i++)
	{
		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)!= SET);
		USART_SendData(USART6,msg[i]);
	}
}










