/*
 * 04ledbutton_ext.c
 *
 *  Created on: Jul 27, 2024
 *      Author: shilpa
 */

#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "stdint.h"
#include "string.h"

#define HIGH		1
#define	LOW			0
#define BTN_PRESSED	LOW

void delay(void)
{
	for(uint32_t i=0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	//this is LED toggle gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;		//external LED config pin
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	//this is button gpio configuration
	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);

	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_6, GPIO_PIN_RESET);
	//IRQ confiuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI4, NVIC_IRQ_PR15);
	GPIO_IRQConfig(IRQ_NO_EXTI4, ENABLE);

	while(1);
	return 0;
}

void EXTI4_IRQHandler(void)
{
	delay();		//200msec
	GPIO_IRQHandler(GPIO_PIN_NO_4);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
}
