/*
 * 07i2c_master_tx_testing.c
 *
 *  Created on: Aug 10, 2024
 *      Author: shilpa
 */

#include "stm32f446xx.h"
#include "stdio.h"
#include "string.h"

extern void initialise_monitor_handles();

I2C_Handle_t I2C1Handle;
#define MY_ADDR		0x61
#define SLAVE_ADDR 	0X68

//data receive buffer
uint8_t rcv_buffer[32];

void delay(void)
{
	for(uint32_t i=0; i<500000/2; i++);
}
/*
 * PB6 -> I2C1_SCL
 * PB9 -> I2C1_SDA
 */
void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//configure serial clock
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);

	//configure serial data
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = (uint8_t)I2C_SCL_SPEED_FM4K;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn, GPIOLed;
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led GPIO configuration
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLed);
}
int main(void)
{
	uint8_t commandcode, len;

	initialise_monitor_handles();
	printf("Application is running");

	GPIO_ButtonInit();

	//I2C pin initialization
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAck(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200msc of delay
		delay();

		commandcode = 0x51;

		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR);

		I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR);

		commandcode = 0x52;

		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR);

		I2C_MasterReceiveData(&I2C1Handle, rcv_buffer, len, SLAVE_ADDR);

		rcv_buffer[len+1] = '\0';
		printf("Data : %s", rcv_buffer);

	}


}
