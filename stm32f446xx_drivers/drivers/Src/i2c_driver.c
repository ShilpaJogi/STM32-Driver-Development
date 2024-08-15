/*
 * i2c_driver.c
 *
 *  Created on: Aug 6, 2024
 *      Author: shilpa
 */

#include "i2c_driver.h"

uint16_t AHB_PreScaler[9] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; //bit for read or write
	SlaveAddr &= ~(1);		//slave address + r/w bit=0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; //bit for read or write
	SlaveAddr |= 1;		//slave address + r/w bit=1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t read;
//	read = pI2Cx->SR2;
//	(void)read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		if(pI2CHandle->TxRxstate == I2C_BUSY_IN_RX)
		{
			//device is master mode
			if(pI2CHandle->Rxsize == 1)
			{
				//disable acknowledge
				I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR flag
				read = pI2CHandle->pI2Cx->SR1;
				read = pI2CHandle->pI2Cx->SR2;
				(void)read;
			}
		}
		else
		{
			//clear the ADDR flag
			read = pI2CHandle->pI2Cx->SR1;
			read = pI2CHandle->pI2Cx->SR2;
			(void)read;
		}
	}else
	{
		//device is slave mode
		//clear the ADDR flag
		read = pI2CHandle->pI2Cx->SR1;
		read = pI2CHandle->pI2Cx->SR2;
		(void)read;
	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
/**************************************************************
 * fn			: I2C_SlaveEnDiCallbackEvents
 * brief		: this function enables or disables callback
 * 				  events
 * param[1]		: base address of I2C peripheral
 * param[2]		: ENABLE or DISABLE macros
 * return		: None
 * note			: None
 */
void I2C_SlaveEnDiCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}
/**************************************************************
 * fn			: I2C_PeriClockControl
 * brief		: this function enables or disables peripheral
 * 				  clock for the given I2C port
 * param[1]		: base address of I2C peripheral
 * param[2]		: ENABLE or DISABLE macros
 * return		: None
 * note			: None
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}
uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}
/**************************************************************************
 * fn			: RCC_GetPCLK1Value
 * brief		: this function reads the system clock value
 * param[1]		: I2C reset handler
 * return		: None
 * note			: None
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);		//get 2 bits to the LSB position and mask
	if(clksrc == 0)
	{
		SystemClk = 16000000;	//HSI is 16MHz
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;	//HSE is 8MHz
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	//for AHB pre_scaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}
	//for APB1 pre_scaler
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;		//formula to calculate clock

	return pclk1;
}
/*************************************************************************
 * fn			: I2C_Init
 * brief		: this function configures the I2C mode(standard or fast),
 * 				  speed, device address, Ack state, rise time for i2c pin
 * param[1]		: I2C pin configuration handler
 * return		: None
 * note			: None
 *************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;

	//enable the clock for the I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//acknowledge bit control
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure FREQ(clock frequency) field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//configure the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);				//requirement says set this bit always 1
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//configure the clock control register(CCR) calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast
		tempreg |= (1 << 15);		//master mode selection bit is set which is Fm
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);	//configure fast mode duty cycle
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE rise time Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard
		tempreg = (RCC_GetPCLK1Value() / 1000000U ) + 1;
	}else
	{
		//mode is fast
		tempreg = ((RCC_GetPCLK1Value() * 300)/ 1000000U ) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}
/**************************************************************
 * fn			: I2C_DeInit
 * brief		: this function resets the all I2C peripheral
 * param[1]		: I2C reset handler
 * return		: None
 * note			: None
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}
/**************************************************************
 * fn			: I2C_MasterSendData
 * brief		: this function sends the data to slave
 * param[4]		: I2C add, Tx Buffer, Length, slave address
 * return		: None
 * note			: None
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm the start generation is completed by checking the SB flag in the SR1
	//Note : until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	//3. send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4.confirm that address phase is completed by checking the ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	//5. clear the ADDR flag according to its software sequence
	//Note: until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send data until len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );	//wait until TxE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when len becomes 0 wait for TxE=1 & BTF=1 before generating the STOP condition
	//Note: TxE=1 & BTF=1(SCL will be stretched), means both SR & DR are empty and next transmission should begin
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );

	//8. generate STOP condition & master need not to wait for the completion of stop condition
	//Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}
/**************************************************************
 * fn			: I2C_MasterReceiveData
 * brief		: this function Receives the data from slave
 * param[4]		: I2C add, Rx Buffer, Length, slave address
 * return		: None
 * note			: None
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm the start generation is completed by checking SB flag in SR1
	//Note: until SB is cleared SCL will be stretched to LOW
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	//3. send the address of the slave with r/w bit R(1) total 8bits
	I2C_ExecuteAddressRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//disable acknowledge
		I2C_ManageAck(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );	//wait until RXNE is set

		//generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data into buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}

	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the address flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until length becomes zero
		for(uint32_t i=Len; i>0; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

			if(i == 2)		//if last 2 bytes are remaining
			{
				//disable ack bit
				I2C_ManageAck(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//read the data from data register into buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxbuffer++;
		}
	}

	//re-enable acknowledge bit
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAck(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}
/**************************************************************
 * fn			: I2C_SlaveSendData
 * brief		: this function send the data to master
 * param[1]		: *pI2C -> reg definition
 * param[2]		: Data to be sent
 * return		: None
 * note			: None
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t Data)
{
	pI2C->DR = Data;
}
/**************************************************************
 * fn			: I2C_SlaveReceiveData
 * brief		: this function Receives the data from master
 * param[1]		: receiver
 * return		: data
 * note			: None
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}
/**************************************************************
 * fn			: I2C_IRQConfig
 * brief		: this function is for IRQ register configuration
 * param[1]		: IRQ Number
 * param[2]		: Enable/Disable macro
 * return		: None
 * note			: None
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)	//32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
		{
			//program ISER2 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER1 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)	//32 to 63
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
		{
			//program ICER2 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 64));
		}
	}
}
/**************************************************************
 * fn			: I2C_IRQPriorityConfig
 * brief		: this function is for IRQ priority configuration
 * param[1]		: IRQ Number
 * param[2]		: IRQ Priority
 * return		: None
 * note			: None
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);
}
/**************************************************************
 * fn			: I2C_PeripheralControl
 * brief		: this function control the Enable/Disables
 * param[2]		: I2C add, Enable/Disable
 * return		: None
 * note			: This is blocking or polling call function
 * 				: Here were polling for the TXE flag to SET
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
/**************************************************************
 * fn			: I2C_GetFlagStatus
 * brief		: this function read the flag status
 * param[1]		: I2C reset handler
 * return		: None
 * note			: None
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagname)
{
	if(pI2Cx->SR1 & Flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ManageAck(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxstate;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxstate = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxstate;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxstate = I2C_BUSY_IN_RX;
		pI2CHandle->Rxsize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}
	return busystate;
}

static void I2C_MasterRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->Rxsize == 1)
	{
		//1.receive  data from DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		//2. decrement the Rx length
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->Rxsize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack
			I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
		}
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0)
	{
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//close receive data
		I2C_CloseReceiveData(pI2CHandle);

		//notify application
		I2C_AppEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

static void I2C_MasterTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the Tx length
		pI2CHandle->TxLen--;

		//3. increment the buffer address
		pI2CHandle->pTxBuffer++;


	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. handle for intrerupt generated by SB event
	//Note: SB flag applicable in master mode only
	if(temp1 && temp3)
	{
		//SB flag is set
		if(pI2CHandle->TxRxstate == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxstate == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. handle for interrupt generated by ADDR event
	//Note: master mode - address sent/slave mode - address matched with own address
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3.handle for interrpt generated by BTF(byte transfer finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxstate == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE))		//make sure TXE is set
			{
				if(pI2CHandle->TxLen == 0){
					//generate the stop condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//notify the application about transmission complete
					I2C_AppEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxstate == I2C_BUSY_IN_RX)
		{
				;
		}

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. handle for the interrpt generated by STOP event
	//Note: stop detection flag applicable in slave mode
	if(temp1 && temp3)
	{
		//STOP flag is set
		//clear STOPF read SR1 write CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify application that STOP is detected
		I2C_AppEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
	//5. handle for interrpt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))		//check for device mode
		{
			//TXE flag is set
			if(pI2CHandle->TxRxstate == I2C_BUSY_IN_TX)
			{
				I2C_MasterTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave mode - make sure that slave is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_AppEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	//6. handle for interrpt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))		//check for device mode
		{
			//RXNE flag is set
			if(pI2CHandle->TxRxstate == I2C_BUSY_IN_RX)
			{
				I2C_MasterRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave mode - make sure that slave is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_AppEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_AppEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable ITBUFEN control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//implement the code to disable ITEVFEN control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxstate = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->Rxsize = 0;
	I2C_ManageAck(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable ITBUFEN control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//implement the code to disable ITEVFEN control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxstate = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}
