/*
 * gpio_driver.h
 *
 *  Created on: Jul 11, 2024
 *      Author: shilpa
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include <stdint.h>
#include"stm32f446xx.h"

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/******handle structure for GPIO pin********/
typedef struct{
	GPIO_RegDef_t *pGPIOx;	//this holds the base address of GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//this holds GPIO pin config setting
}GPIO_Handle_t;

/*******GPIO pin numbers*************/
#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15

/******GPIO pin possible modes********/
#define GPIO_MODE_IN		0
#define	GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define	GPIO_MODE_ANALOG	3
#define	GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/**********GPIO pin possible output types***********/
#define	GPIO_OP_TYPE_PP		0
#define	GPIO_OP_TYPE_OD		1

/**********GPIO pin possible output speed types***********/
#define	GPIO_SPEED_LOW		0
#define	GPIO_SPEED_MED		1
#define	GPIO_SPEED_FAST		2
#define	GPIO_SPEED_HIGH		3

/**********GPIO pin pull up pull down configuration types***********/
#define	GPIO_NO_PUPD		0
#define	GPIO_PIN_PU			1
#define	GPIO_PIN_PD			2

/****************************************************************************
*				APIs supported by this driver
*		For more information about the APIs check the function definitions
****************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);				//Init & De Init
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);	//peripheral clock setup
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);	//Data read and write
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi); //IRQ configuration and ISR Handling
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNumber);



#endif /* INC_GPIO_DRIVER_H_ */
