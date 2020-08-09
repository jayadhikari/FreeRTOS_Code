#include "myPrototypes.h"
#include "stdio.h"
//#include "stdint.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>


static void prvSetupUART(void);
static void prvSetupLEDs(void);
static void prvSetupButton(void);


//This function will do all HW related settings
void vSetupHardware(void)
{
	prvSetupUART();
	prvSetupLEDs();
	prvSetupButton();
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

//discovery board user button is on PA0
static void prvSetupButton(void)
{
	GPIO_InitTypeDef  gpioConfig;
	EXTI_InitTypeDef extiConfig;

	//enable GPIOD peripheral bus clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	//enable clock for SYSCFG block. This controls the exti interrupt
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	//clear GPIO config structure
	memset(&gpioConfig,0,sizeof(gpioConfig));
	memset(&extiConfig,0,sizeof(extiConfig));
	//set PA0
	gpioConfig.GPIO_Pin = GPIO_Pin_0;
	gpioConfig.GPIO_Mode = GPIO_Mode_IN;	//set alternate function mode
	gpioConfig.GPIO_OType = GPIO_OType_PP;
	gpioConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;	//enable pull up on both pins
	GPIO_Init(GPIOA, &gpioConfig);			//set the pin config

	//setup interrupt for button
	//setup external interrupt controller block (EXTI)
/*	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	//Setup EXTI Line
	extiConfig.EXTI_Line = EXTI_Line0;
	extiConfig.EXTI_Mode = EXTI_Mode_Interrupt;
	extiConfig.EXTI_Trigger = EXTI_Trigger_Falling;
	extiConfig.EXTI_LineCmd = ENABLE;
	EXTI_Init(&extiConfig);

	//Setup NVIC for IRQ number
	//priority and IRQ number can be found in Vector table for STM32F405xx/07xx and STM32F415xx/17xx
	NVIC_SetPriority(EXTI0_IRQn,5);
	//enable button IRQ
	NVIC_EnableIRQ(EXTI0_IRQn);
	*/
}

//discovery board has 4 leds
	//GREEN LED on PD12
	//ORANGE LED on PD13
	//RED LED on PD14
	//BLUE LED on PD15
static void prvSetupLEDs(void)
{
	GPIO_InitTypeDef  gpioConfig;
	//enable GPIOD peripheral bus clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	//clear GPIO config structure
	memset(&gpioConfig,0,sizeof(gpioConfig));
	//set PD12,PD13,PD14,PD15 as GPIOs
	gpioConfig.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;//set both 6 and 7 as GPIO
	gpioConfig.GPIO_Mode = GPIO_Mode_OUT;	//set alternate function mode
	gpioConfig.GPIO_OType = GPIO_OType_PP;
	gpioConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;	//enable pull up on both pins
	GPIO_Init(GPIOD, &gpioConfig);			//set the pin config

}


void toggleLED(void)
{
	GPIO_ToggleBits(GPIOD,GPIO_Pin_12);
	GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
	GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
	GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
}
void setLED(void)
{
	GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_SET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_SET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
}
void resetLED(void)
{
	GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
}
