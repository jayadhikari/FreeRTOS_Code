#include "myPrototypes.h"
#include "stdio.h"
//#include "stdint.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
uint8_t buttonPressedFlag =RESET;


TaskHandle_t xButtonHandle = NULL;
TaskHandle_t xLEDHandle = NULL;

char msg[100];

void vTaskButton(void *params)
{
	for(;;)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == SET)
		{
			//crude blocking debounce
			rtos_delay(100);
			//send notification to LED task
			xTaskNotify(xLEDHandle,0,eIncrement);
			printMsg("Button Task\n\r");
		}

	}
}
void vTaskLED(void *params)
{
	uint32_t notificationCount =0;

	for(;;)
	{
		//wait for notification. till then don't execute
		//ulBitsToClearOnEntry, ulBitsToClearOnExit,*pulNotificationValue, xTicksToWait
		if(xTaskNotifyWait(0,0,&notificationCount,portMAX_DELAY) == pdTRUE)
		{
			sprintf(msg,"Notification count : %ld\n\r",notificationCount);
			toggleLED();
			printMsg(msg);
		}
		printMsg("LED Task\n\r");

	}
}

void printMsg(char *msg)
{
	for(uint32_t i=0;i<strlen(msg);i++)
	{
		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)!= SET);
		USART_SendData(USART6,msg[i]);
	}
}

//button interrupt handler
/*
void EXTI0_IRQHandler(void)
{
	//clear the interrupt pending bit for EXTI line 0
	EXTI_ClearITPendingBit(EXTI_Line0);
	buttonPressedFlag ^= 1;
}
*/
void rtos_delay(uint32_t delayInMS)
{
	uint32_t currentTickCount = xTaskGetTickCount();
	uint32_t delayInTicks = (delayInMS * configTICK_RATE_HZ)/1000;

	while(xTaskGetTickCount() < (currentTickCount + delayInTicks));
}



