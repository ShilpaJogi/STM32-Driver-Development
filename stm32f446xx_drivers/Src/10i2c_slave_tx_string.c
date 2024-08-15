/*
 * 09i2c_master_rx_ITtesting.c
 *
 *  Created on: Aug 11, 2024
 *      Author: shilpa
 */

#include "stm32f446xx.h"
#include "stdio.h"
#include "string.h"


I2C_Handle_t I2C1Handle;

#define SLAVE_ADDR 	0X68
#define MY_ADDR 	SLAVE_ADDR
//data transmit buffer
uint8_t Tx_buf[32] = "STM32 Slave Mode Testing..";

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

	GPIO_ButtonInit();

	//I2C pin initialization
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ configuration
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnDiCallbackEvents(I2C1, ENABLE);

	//enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAck(I2C1, I2C_ACK_ENABLE);

	while(1);

}
void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}
void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	static uint8_t commandcode = 0;
	static uint8_t count = 0;
	if(AppEvent == I2C_EV_DATA_REQ)
	{
		//master wants some data, slave has to send it
		if(commandcode == 0x51)
		{
			//send length information to master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));
		}else if(commandcode == 0x52)
		{
			//send the content of Tx
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[count++]);
		}
	}else if(AppEvent == I2C_EV_DATA_RCV)
	{
		//data is waiting for the slave to read, slave has to read it
		commandcode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}else if(AppEvent == I2C_ERROR_AF)
	{
		//this happen only during slave Tx
		//master has sent the NACK, so slave should understand the master doesn't need data
		commandcode = 0xff;
		count = 0;

	}else if(AppEvent == I2C_EV_STOP)
	{
		//this happens during slave Rx
		//master has ended the I2C comm with the slave

	}
}
