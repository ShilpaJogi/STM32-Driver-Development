/*
 * spi_driver.c
 *
 *  Created on: Aug 3, 2024
 *      Author: shilpa
 */
#include "spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_overerr_interrupt_handle(SPI_Handle_t *pSPIHandle);
/**************************************************************
 * fn			: SPI_PeriClockControl
 * brief		: this function enables or disables peripheral
 * 				  clock for the given SPI port
 * param[1]		: base address of SPI peripheral
 * param[2]		: ENABLE or DISABLE macros
 * return		: None
 * note			: None
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1){
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_EN();
			}
		}
		else{
			if(pSPIx == SPI1){
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_DI();
			}
		}
}

/*******************************************************************
 * fn			: SPI_Init
 * brief		: this function configures the SPI mode, speed
 * 				  frame format, bus config, clock polarity & phase
 * param[1]		: SPI pin configuration handler
 * return		: None
 * note			: None
 *******************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/*first lets configure the SPI CR1 register*/
	uint32_t	tempreg=0;

	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/*configure the device mode master/slave*/
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	/*Configure the bus Half/Full duplex*/
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/*Bi-directional mode should be cleared*/
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/*Bi-directional mode should be set*/
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY)
	{
		/*Bi-directional mode should be cleared*/
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		/*RXONLY bit must be set*/
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	/*Configure the spi serial clock speed(baud rate)*/
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BD;

	/*Configure the data frame format DFF*/
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/*Configure the CPOL clock polarity*/
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/*Configure the CPHA clock phase*/
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/**************************************************************
 * fn			: SPI_DeInit
 * brief		: this function resets the all SPI peripheral
 * param[1]		: SPI reset handler
 * return		: None
 * note			: None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}
/**************************************************************
 * fn			: SPI_GetFlagStatus
 * brief		: this function read the flag status
 * param[1]		: SPI reset handler
 * return		: None
 * note			: None
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagname)
{
	if(pSPIx->SR & Flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/**************************************************************
 * fn			: SPI_SendData
 * brief		: this function sends the data to slave
 * param[3]		: SPI add, Tx Buffer, Length
 * return		: None
 * note			: This is blocking or polling call function
 * 				: Here were polling for the TXE flag to SET
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t temp=0;
	while(Len > 0)
	{
		//wait until TXE is set
		while( ! (pSPIx->SR & (1 << 1)) );
//		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2.check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit DFF
			//1.load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
	//wait until TXE is set
	while( ! (pSPIx->SR & (1 << 1)) );

	//wait for busy flag to reset
	while((pSPIx->SR & (SPI_BUSY_FLAG)) );

	/*Clear Over flow flag*/
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;
}
/**************************************************************
 * fn			: SPI_ReceiveData
 * brief		: this function receives the data from slave
 * param[3]		: SPI add, Rx Buffer, Length
 * return		: None
 * note			: This is blocking or polling call function
 * 				: Here were polling for the RXNE flag to SET
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//wait until RXNE is set
			while( ! (pSPIx->SR & (1 << 1)) );
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			//2.check the DFF bit in CR1
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF) )
			{
				//16 bit DFF
				//1.load the data from DR to Rx buffer address
			    *((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*pRxBuffer = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}
		}
		//wait until TXE is set
//		while( ! (pSPIx->SR & (1 << 1)) );

		//wait for busy flag to reset
//		while((pSPIx->SR & (SPI_BUSY_FLAG)) );

		/*Clear Over flow flag*/
//		temp = pSPIx->DR;
//		temp = pSPIx->SR;
}
/**************************************************************
 * fn			: SPI_PeripheralControl
 * brief		: this function control the Enable/Disables
 * param[2]		: SPI add, Tx Buffer, Length
 * return		: None
 * note			: This is blocking or polling call function
 * 				: Here were polling for the TXE flag to SET
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
/**************************************************************
 * fn			: SPI_SSIConfig
 * brief		: this function control the Enable/Disables
 * param[2]		: SPI add, Tx Buffer, Length
 * return		: None
 * note			: This is blocking or polling call function
 * 				: Here were polling for the TXE flag to SET
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/**************************************************************
 * fn			: SPI_IRQConfig
 * brief		: this function is for IRQ register configuration
 * param[1]		: IRQ Number
 * param[2]		: Enable/Disable macro
 * return		: None
 * note			: None
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * fn			: SPI_IRQPriorityConfig
 * brief		: this function is for IRQ priority configuration
 * param[1]		: IRQ Number
 * param[2]		: IRQ Priority
 * return		: None
 * note			: None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;
		uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

		*(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);
}
/**************************************************************************************
 * fn			: SPI_SendDataIT
 * brief		: this function send the data to slave when interrupt came
 * param[3]		: SPI add, Tx Buffer, Length
 * return		: return the state
 * note			: This is blocking or polling call function
 * 				: Here were polling for the TXE flag to SET
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. save the TX buffer address & len info in global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. mark the SPI state is busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable TXEIE(empty interrupt enable) control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		//4. Data transmission will be handled by the ISR code
	}
	return state;
}
/*************************************************************************************
 * fn			: SPI_ReceiveDataIT
 * brief		: this function receives the data from slave when interrupt came
 * param[3]		: SPI add, Rx Buffer, Length
 * return		: return the state
 * note			: This is blocking or polling call function
 * 				: Here were polling for the RXNE flag to SET
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
			//1. save the Rx buffer address & len info in global variable
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;

			//2. mark the SPI state is busy in reception so that no other code can take over same SPI peripheral until reception is over
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//3. Enable RXNEIE(Not empty interrupt enable) control bit to get interrupt whenever RXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
			//4. Data transmission will be handled by the ISR code
		}
		return state;
}
/**************************************************************
 * fn			: SPI_IRQHandler
 * brief		: this function read the GPIO port
 * param[1]		:
 * return		: None
 * note			: None
 */
void SPI_IRQHandler(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	//check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << (SPI_SR_TXE));
	temp2 = pHandle->pSPIx->CR2 & (1 << (SPI_CR2_TXEIE));
	if(temp1 & temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << (SPI_SR_RXNE));
	temp2 = pHandle->pSPIx->CR2 & (1 << (SPI_CR2_RXNEIE));
	if(temp1 & temp2)
	{
		spi_rxe_interrupt_handle(pHandle);
	}

	//check for over flag
	temp1 = pHandle->pSPIx->SR & (1 << (SPI_SR_OVR));
	temp2 = pHandle->pSPIx->CR2 & (1 << (SPI_CR2_ERRIE));
	if(temp1 & temp2)
	{
		spi_overerr_interrupt_handle(pHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		//16 bit DFF
		//1.load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	//TxLen is 0, close the SPI transmission and inform app that TX is over
	if(! pSPIHandle->TxLen)
	{
		//this prevents interrupts from setting up TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
		if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit DFF
			//1.load the data into the DR
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR ;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer--;
			pSPIHandle->pRxBuffer--;
		}else
		{
			//8 bit DFF
			*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR ;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
		}
		//TxLen is 0, close the SPI reception and inform app that RX is over
		if(! pSPIHandle->RxLen)
		{
			//this prevents interrupts from setting up RXNE flag
			SPI_CloseReception(pSPIHandle);
			SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}
static void spi_overerr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the over flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	//2. inform the Application
	SPI_AppEventCallback(pSPIHandle, SPI_EVENT_OVER_ERR);
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
void SPI_ClearOVERFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
__attribute__((weak)) void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//this is weak implementation, the application may override this function
}
