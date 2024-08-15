/*
 * stm32f446xx.h
 *
 *  Created on: Jul 8, 2024
 *      Author: shilpa
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include "stdint.h"
#include "stddef.h"

#define __weak __attribute__((weak))
/*****************processor specific details************************/
/*
 * ARM cortex M4 processor NVIC ISERx register address
 */
#define NVIC_ISER0			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1			((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2			((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3			((volatile uint32_t*)0xE000E10C)

/*
 * ARM cortex M4 processor NVIC ICERx register address
 */
#define NVIC_ICER0			((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2			((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0xE000E18C)
/*
 * ARM cortex M4 processor NVIC ICERx register address
 */
#define NVIC_PR_BASE_ADDR	((volatile uint32_t*)0xE000E400)
//ARM cortex Mx processor no.of priority bit implemented in priority register
#define NO_PR_BITS_IMPLEMENTED		4


#define ENABLE	        1
#define DISABLE	        0
#define SET	            ENABLE
#define RESET	        DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET 		RESET
#define FLAG_SET		SET

/***********Base address of flash and ROM**************/
#define FLASH_BASEADDR		0X08000000U
#define	SRAM1_BASEADDR		0X20000000U
#define SRAM2_BASEADDR		0X20001C00U
#define	ROM					0x1fff0000U
#define SRAM				SRAM1_BASEADDR

/**********Base address of peripherals****************/
#define	PERIPH_BASE			0X40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define	APB2PERIPH_BASE		0X40010000U
#define	AHB1PERIPH_BASE		0X40020000U
#define	AHB2PERIPH_BASE		0x50000000U

/*******Base address of peripherals which are hanging on AHB1 bus******/
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0X0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0X0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0X0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0X0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0X1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0X1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0X1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0X1C00)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

/*******Base address of peripherals which are hanging on APB1 bus******/
#define	I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define	I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define	I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00)
#define	SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define	SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)
#define	USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define	USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define	UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define	UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)

/*******Base address of peripherals which are hanging on APB2 bus******/
#define	SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define	USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define	USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define	EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00)
#define	SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)

/*******Peripheral definitions(peripheral base address typecast)******/
#define	GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define	GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define	GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define	GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define	GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define	GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define	GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define	GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define	SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define	SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define	SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

/********Macros to reset the GPIO peripherals************/
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)

/********Macros to reset the SPI peripherals************/
#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 15));} while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));} while(0)

/********Macros to reset the I2C peripherals************/
#define I2C1_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));} while(0)
#define I2C2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));} while(0)
#define I2C3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));} while(0)

/***returns port code for given GPIOx base address*****/
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 : 0)

/*******Clock enable macros for GPIOx peripherals********/
#define	GPIOA_PCLK_EN() 	   (RCC->AHB1ENR |= (1 << 0) )
#define	GPIOB_PCLK_EN() 	   (RCC->AHB1ENR |= (1 << 1) )
#define	GPIOC_PCLK_EN() 	   (RCC->AHB1ENR |= (1 << 2) )
#define	GPIOD_PCLK_EN() 	   (RCC->AHB1ENR |= (1 << 3) )
#define	GPIOE_PCLK_EN() 	   (RCC->AHB1ENR |= (1 << 4) )
#define	GPIOF_PCLK_EN() 	   (RCC->AHB1ENR |= (1 << 5) )
#define	GPIOG_PCLK_EN() 	   (RCC->AHB1ENR |= (1 << 6) )
#define	GPIOH_PCLK_EN() 	   (RCC->AHB1ENR |= (1 << 7) )

/*******Clock enable macros for I2Cx peripherals********/
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23) )

/*******Clock enable macros for SPIx peripherals********/
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13) )

/*******Clock enable macros for USARTx peripherals********/
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4) )
#define UART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17) )
#define UART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5) )

/*******Clock enable macros for SYSCFG peripherals********/
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14) )

/*******Clock disable macros for GPIOx peripherals********/
#define	GPIOA_PCLK_DI() 	   (RCC->AHB1ENR &= ~(1 << 0) )
#define	GPIOB_PCLK_DI() 	   (RCC->AHB1ENR &= ~(1 << 1) )
#define	GPIOC_PCLK_DI() 	   (RCC->AHB1ENR &= ~(1 << 2) )
#define	GPIOD_PCLK_DI() 	   (RCC->AHB1ENR &= ~(1 << 3) )
#define	GPIOE_PCLK_DI() 	   (RCC->AHB1ENR &= ~(1 << 4) )
#define	GPIOF_PCLK_DI() 	   (RCC->AHB1ENR &= ~(1 << 5) )
#define	GPIOG_PCLK_DI() 	   (RCC->AHB1ENR &= ~(1 << 6) )
#define	GPIOH_PCLK_DI() 	   (RCC->AHB1ENR &= ~(1 << 7) )

/*******Clock disable macros for I2Cx peripherals********/
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23) )

/*******Clock disable macros for SPIx peripherals********/
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13) )

/*******Clock disable macros for USARTx peripherals********/
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4) )
#define UART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17) )
#define UART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5) )

/*******Clock disable macros for SYSCFG peripherals********/
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14) )

/*******Peripheral register definition structure for GPIO********/
typedef struct{
	volatile uint32_t	MODER;			/* offset : 0x00 */
	volatile uint32_t	OTYPER;			/* offset : 0x04 */
	volatile uint32_t	OSPEEDR;		/* offset : 0x08 */
	volatile uint32_t	PUPDR;			/* offset : 0x0C */
	volatile uint32_t	IDR;			/* offset : 0x10 */
	volatile uint32_t	ODR;			/* offset : 0x14 */
	volatile uint32_t	BSRR;			/* offset : 0x18 */
	volatile uint32_t	LCKR;			/* offset : 0x1C */
	volatile uint32_t	AFR[2];			/* offset : 0x20 */
}GPIO_RegDef_t;

/*******Peripheral register definition structure for RCC********/
typedef struct{
	volatile uint32_t CR;				/*offset : 0x00 */
	volatile uint32_t PLLCFGR;			/*offset : 0x04 */
	volatile uint32_t CFGR;				/*offset : 0x08 */
	volatile uint32_t CIR;				/*offset : 0x0C */
	volatile uint32_t AHB1RSTR;			/*offset : 0x10 */
	volatile uint32_t AHB2RSTR;			/*offset : 0x14 */
	volatile uint32_t AHB3RSTR;			/*offset : 0x18 */
			 uint32_t RESERVED0;		/*offset : 0x1C */
	volatile uint32_t APB1RSTR;			/*offset : 0x20 */
	volatile uint32_t APB2RSTR;			/*offset : 0x24 */
			 uint32_t RESERVED1;		/*offset : 0x28 */
			 uint32_t RESERVED2;		/*offset : 0x2C */
	volatile uint32_t AHB1ENR;			/*offset : 0x30 */
	volatile uint32_t AHB2ENR;			/*offset : 0x34 */
	volatile uint32_t AHB3ENR;			/*offset : 0x38 */
			 uint32_t RESERVED3;		/*offset : 0x3C	*/
	volatile uint32_t APB1ENR;			/*offset : 0x40 */
	volatile uint32_t APB2ENR;			/*offset : 0x44 */
			 uint32_t RESERVED4;		/*offset : 0x48 */
			 uint32_t RESERVED5;		/*offset : 0x4C */
	volatile uint32_t AHB1LPENR;		/*offset : 0x50 */
	volatile uint32_t AHB2LPENR;		/*offset : 0x54 */
	volatile uint32_t AHB3LPENR;		/*offset : 0x58 */
			 uint32_t RESERVED6;		/*offset : 0x5C */
	volatile uint32_t APB1LPENR;		/*offset : 0x60 */
	volatile uint32_t APB2LPENR;		/*offset : 0x64 */
			 uint32_t RESERVED7;		/*offset : 0x68 */
			 uint32_t RESERVED8;		/*offset : 0x6C */
	volatile uint32_t BDCR;				/*offset : 0x70 */
	volatile uint32_t CSR;				/*offset : 0x74 */
	         uint32_t RESERVED9;		/*offset : 0x78 */
			 uint32_t RESERVED10;		/*offset : 0x7C */
	volatile uint32_t SSCGR;			/*offset : 0x80 */
	volatile uint32_t PLLI2SCFGR;		/*offset : 0x84 */
	volatile uint32_t PLLSAICFGR;		/*offset : 0x88 */
	volatile uint32_t DCKCFGR;			/*offset : 0x8C */
	volatile uint32_t CKGATENR;			/*offset : 0x90 */
	volatile uint32_t DCKCFGR2;			/*offset : 0x94 */
}RCC_RegDef_t;

/*******Peripheral register definition structure for EXTI********/
typedef struct{
	volatile uint32_t	IMR;			/* offset : 0x00 */
	volatile uint32_t	EMR;			/* offset : 0x04 */
	volatile uint32_t	RTSR;			/* offset : 0x08 */
	volatile uint32_t	FTSR;			/* offset : 0x0C */
	volatile uint32_t	SWIER;			/* offset : 0x10 */
	volatile uint32_t	PR;				/* offset : 0x14 */
}EXTI_RegDef_t;

/*******Peripheral register definition structure for SYSCFG********/
typedef struct{
	volatile uint32_t	MEMRMP;			/* offset : 0x00 */
	volatile uint32_t	PMC;			/* offset : 0x04 */
	volatile uint32_t	EXTICR[4];		/* offset : 0x08 - 0x14 */
	         uint32_t   RESERVED[2];	/* offset : 0x18 & 0x1C reserved */
	volatile uint32_t	CMPCR;			/* offset : 0x20 */
			 uint32_t   RESERVED1[2];	/* offset : 0x24 & 0x28 reserved */
	volatile uint32_t	CFGR;			/* offset : 0x2C */
}SYSCFG_RegDef_t;

/*******Peripheral register definition structure for SPI********/
typedef struct{
	volatile uint32_t	CR1;			/* offset : 0x00 control register*/
	volatile uint32_t	CR2;			/* offset : 0x04 control register*/
	volatile uint32_t	SR;				/* offset : 0x08 status register*/
	volatile uint32_t   DR;				/* offset : 0x0C Data register*/
	volatile uint32_t	CRCPR;			/* offset : 0x10 CRC polynomial register*/
	volatile uint32_t   RXCRCR;			/* offset : 0x14 Rx CRC register*/
	volatile uint32_t	TXCRCR;			/* offset : 0x18 Tx CRC register*/
	volatile uint32_t   I2SCFGR;		/* offset : 0x1C I2S Configuration register*/
	volatile uint32_t	I2SPR;			/* offset : 0x20 I2S Prescaler register*/
}SPI_RegDef_t;

/*******Peripheral register definition structure for I2C********/
typedef struct{
	volatile uint32_t	CR1;			/* offset : 0x00 control register*/
	volatile uint32_t	CR2;			/* offset : 0x04 control register*/
	volatile uint32_t	OAR1;			/* offset : 0x08 own address register*/
	volatile uint32_t   OAR2;			/* offset : 0x0C own address register*/
	volatile uint32_t	DR;				/* offset : 0x10 Data register*/
	volatile uint32_t   SR1;			/* offset : 0x14 status register*/
	volatile uint32_t	SR2;			/* offset : 0x18 status register*/
	volatile uint32_t   CCR;			/* offset : 0x1C clock control register*/
	volatile uint32_t	TRISE;			/* offset : 0x20 rise time in Fm/Sm mode register*/
	volatile uint32_t	FLTR;			/* offset : 0x24 fault register*/
}I2C_RegDef_t;

/********IRQ(Interrupt request)number*************/
#define IRQ_NO_EXTI0	     6
#define IRQ_NO_EXTI1	     7
#define IRQ_NO_EXTI2	     8
#define IRQ_NO_EXTI3	     9
#define IRQ_NO_EXTI4	     10
#define IRQ_NO_EXTI9_5	     23
#define IRQ_NO_EXTI15_10     40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

/***macros for all possible priority levels*****/
#define NVIC_IRQ_PR0		0
#define NVIC_IRQ_PR1		1
#define NVIC_IRQ_PR2		2
#define NVIC_IRQ_PR3		3
#define NVIC_IRQ_PR4		4
#define NVIC_IRQ_PR5		5
#define NVIC_IRQ_PR6		6
#define NVIC_IRQ_PR7		7
#define NVIC_IRQ_PR8		8
#define NVIC_IRQ_PR9		9
#define NVIC_IRQ_PR10		10
#define NVIC_IRQ_PR11		11
#define NVIC_IRQ_PR12		12
#define NVIC_IRQ_PR13		13
#define NVIC_IRQ_PR14		14
#define NVIC_IRQ_PR15		15

/********************************************************************
 * Bit position definitions of SPI CR1 control register peripheral
 ********************************************************************/
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BD			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/********************************************************************
 * Bit position definitions of SPI CR2 control register peripheral
 *******************************************************************/
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*******************************************************************
 * Bit position definitions of SPI SR status register peripheral
 ******************************************************************/
#define SPI_SR_RXNE			0		/*receive buffer not empty*/
#define SPI_SR_TXE			1		/*transmit buffer empty*/
#define SPI_SR_CHSIDE		2		/*channel side*/
#define SPI_SR_UDR			3		/*under run  flag*/
#define SPI_SR_CRCERR		4		/*CRC error flag*/
#define SPI_SR_MODF			5		/*mode fault*/
#define SPI_SR_OVR			6		/*over run flag*/
#define SPI_SR_BSY			7		/*busy flag*/
#define SPI_SR_FRE			8		/*frame format error*/

/********************************************************************
 * Bit position definitions of I2C CR1 control register peripheral
 ********************************************************************/
#define I2C_CR1_PE			0		//peripheral enable
#define I2C_CR1_SMBUS		1		//SMbus mode
#define I2C_CR1_SMBTYPE		3		//SMBus type
#define I2C_CR1_ENARP		4		//ARP enable
#define I2C_CR1_ENPEC		5		//PEC enable
#define I2C_CR1_ENGC		6		//general call enable
#define I2C_CR1_NOSTRETCH	7		//clock stretch enable
#define I2C_CR1_START		8		//start generation
#define I2C_CR1_STOP		9		//stop generation
#define I2C_CR1_ACK 		10		//acknowledge enable
#define I2C_CR1_POS 		11		//acknowledge/PEC position
#define I2C_CR1_PEC 		12		//packet error checking
#define I2C_CR1_ALERT   	13		//SMBus alert
#define I2C_CR1_SWRST   	15		//software reset

/********************************************************************
 * Bit position definitions of I2C CR2 control register peripheral
 ********************************************************************/
#define I2C_CR2_FREQ		0		//[0:5]peripheral clock frequency
#define I2C_CR2_ITERREN		8		//error interrupt enable
#define I2C_CR2_ITEVTEN		9		//event interrupt enable
#define I2C_CR2_ITBUFEN		10		//buffer interrupt enable
#define I2C_CR2_DMAEN		11		//DMA request enable
#define I2C_CR2_LAST		12		//DMA last transfer

/********************************************************************
 * Bit position definitions of I2C SR1 status register peripheral
 ********************************************************************/
#define I2C_SR1_SB			0		//start bit
#define I2C_SR1_ADDR		1		//address sent
#define I2C_SR1_BTF			2		//byte transfer finished
#define I2C_SR1_ADD10		3		//10-bit header sent
#define I2C_SR1_STOPF		4		//stop detection
#define I2C_SR1_RxNE		6		//data register not empty
#define I2C_SR1_TxE			7		//data register empty
#define I2C_SR1_BERR		8		//bus error
#define I2C_SR1_ARLO		9		// arbitration lost(master mode)
#define I2C_SR1_AF			10		//acknowledge failure
#define I2C_SR1_OVR			11		//overrun/underrun
#define I2C_SR1_PECERR		12		//PEC error in reception
#define I2C_SR1_TIMEOUT		14		//timeout or tlow error
#define I2C_SR1_SMBALERT	15		//SMBus alert

/********************************************************************
 * Bit position definitions of I2C SR2 status register peripheral
 ********************************************************************/
#define I2C_SR2_MSL			0		//master/slave
#define I2C_SR2_BUSY		1		//bus busy
#define I2C_SR2_TRA			2		//transmit/receiver
#define I2C_SR2_GENCALL		4		//general call address
#define I2C_SR2_SMBDEFAULT	5		//SMBus device default register
#define I2C_SR2_SMBHOST		6		//SMbus host register
#define I2C_SR2_DUALF		7		//dual flag
#define I2C_SR2_PEC			8		//[8:15] packet error checking register


/********************************************************************
 * Bit position definitions of I2C CCR clock control register peripheral
 ********************************************************************/
#define I2C_CCR				0		//[0:11]clock control register in Fm/Sm mode
#define I2C_CCR_DUTY		14		//Fm mode duty cycle
#define I2C_CCR_F_S			15		//I2C master mode selection


#include"spi_driver.h"
#include"gpio_driver.h"
#include "i2c_driver.h"

#endif /* INC_STM32F446XX_H_ */
