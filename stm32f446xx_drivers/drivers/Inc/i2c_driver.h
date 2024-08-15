/*
 * i2c_driver.h
 *
 *  Created on: Aug 6, 2024
 *      Author: shilpa
 */

#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_
#include "stm32f446xx.h"

/*
* Configuration structure for I2CX peripheral
*/
typedef struct{
	uint8_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
* Handle structure for I2Cx peripheral
*/
typedef struct{
	I2C_RegDef_t	*pI2Cx;	/*This holds the base address of I2Cx(x=1,2,3) peripheral*/
	I2C_Config_t	I2C_Config;
	uint8_t 		*pTxBuffer;			//to store app Tx buffer address
	uint8_t 		*pRxBuffer;			//to store app Rx buffer address
	uint8_t			TxLen;				//to store Tx length
	uint8_t			RxLen;				//to store Rx length
	uint8_t			TxRxstate;			//to store communication state
	uint8_t			DevAddr;			//to store slave device address
	uint8_t			Rxsize;				//to store Rx size
	uint8_t			Sr;					//to store repeated start value


}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000	//100kbps
#define I2C_SCL_SPEED_FM4K	400000	//400kbps
#define I2C_SCL_SPEED_FM2K	200000	//200kbps

/*
 *@I2C_ACKControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C application states
 */
#define I2C_READY		0
#define	I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2

/*
 * I2C related status flag definitions
 */
#define I2C_FLAG_TXE	 (1 << I2C_SR1_TxE)
#define I2C_FLAG_RXNE	 (1 << I2C_SR1_RxNE)
#define I2C_FLAG_SB		 (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR	 (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF	 (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF   (1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR	 (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO	 (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF      (1 << I2C_SR1_AF)
#define I2C_FLAG_OVER	 (1 << I2C_SR1_OVER)
#define I2C_FLAG_TIMEOUT (1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_STOPF   (1 << I2C_SR1_STOPF)

#define I2C_DISABLE_SR	RESET
#define I2C_ENABLE_SR	SET

/*
 * I2C application event macros
 */
#define	I2C_EV_TX_CMPLT		0
#define	I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR		3
#define I2C_ERROR_ARLO		4
#define I2C_ERROR_AF		5
#define I2C_ERROR_OVR		6
#define I2C_ERROR_TIMEOUT	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9
/***************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check function definitions
 **************************************************************************/
/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send & Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t Data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/*
 *IRQ configuration and ISR handling
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagname);
void I2C_ManageAck(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnDiCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
/*
 * Application callback
 */
void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


#endif /* INC_I2C_DRIVER_H_ */
