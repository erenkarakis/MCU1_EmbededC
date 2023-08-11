/*
 * stm32f407xx.h
 *
 *  Created on: Aug 3, 2023
 *   Author: Taha Eren Karakış
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/********************************************* Processor Specific Details ********************************************
 * 											 																		*
 ********************************************************************************************************************/

/*
 * ISER -> Interrupt set enable register
 * ICER -> Interrupt clear enable register
 * IPR -> Input priority register
 */

/* ARM Cortex M4 Processor NVIC ISERx Addresses and Pointers */

#define NVIC_ISER_BASEADDR 0xE000E100UL
#define NVIC_ISER0 NVIC_ISER_BASEADDR
#define NVIC_ISER1 (NVIC_ISER_BASEADDR + 0x04)
#define NVIC_ISER2 (NVIC_ISER_BASEADDR + 0x08)
#define NVIC_ISER3 (NVIC_ISER_BASEADDR + 0x0C)
#define NVIC_ISER4 (NVIC_ISER_BASEADDR + 0x10)

#define NVIC_ISER_BASEADDR_PTR ((__vo uint32_t *)NVIC_ISER_BASEADDR)
#define NVIC_ISER0_PTR NVIC_ISER_BASEADDR_PTR
#define NVIC_ISER1_PTR ((__vo uint32_t *)NVIC_ISER1)
#define NVIC_ISER2_PTR ((__vo uint32_t *)NVIC_ISER2)
#define NVIC_ISER3_PTR ((__vo uint32_t *)NVIC_ISER3)
#define NVIC_ISER4_PTR ((__vo uint32_t *)NVIC_ISER4)

/* ARM Cortex M4 Processor NVIC ICERx Addresses and Pointers*/

#define NVIC_ICER_BASEADDR 0XE000E180UL
#define NVIC_ICER0 NVIC_ICER_BASEADDR
#define NVIC_ICER1 (NVIC_ICER_BASEADDR + 0x04)
#define NVIC_ICER2 (NVIC_ICER_BASEADDR + 0x08)
#define NVIC_ICER3 (NVIC_ICER_BASEADDR + 0x0C)
#define NVIC_ICER4 (NVIC_ICER_BASEADDR + 0x10)

#define NVIC_ICER_BASEADDR_PTR ((__vo uint32_t *)NVIC_ICER_BASEADDR)
#define NVIC_ICER0_PTR NVIC_ICER_BASEADDR_PTR
#define NVIC_ICER1_PTR ((__vo uint32_t *)NVIC_ICER1)
#define NVIC_ICER2_PTR ((__vo uint32_t *)NVIC_ICER2)
#define NVIC_ICER3_PTR ((__vo uint32_t *)NVIC_ICER3)
#define NVIC_ICER4_PTR ((__vo uint32_t *)NVIC_ICER4)

/* ARM Cortex M4 Processor NVIC IPRx Addresses and Pointers*/

#define NVIC_IPR_BASEADDR 0xE000E400UL

#define NVIC_IPR_BASEADDR_PTR ((__vo uint32_t *)NVIC_IPR_BASEADDR)

#define NVIC_NO_PR_BITS_IMPLEMENTED 4

/********************************************* Some generic macros *********************************************/

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define HIGH ENABLE
#define LOW DISABLE
#define FLAG_RESET RESET
#define FLAG_SET SET

/* Base addresses of memories */
#define FLASH_BASEADDR 0x08000000UL
#define SRAM1_BASEADDR 0x20000000UL
#define SRAM2_BASEADDR 0x2001C000UL
#define ROM_BASEADDR 0x1FFF0000UL
#define SRAM_BASEADDR SRAM1_BASEADDR

/* AHBx and APBx bus peripheral base addresses */
#define PERIPH_BASEADDR 0x40000000UL
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0x40010000UL
#define AHB1PERIPH_BASEADDR 0x40020000UL
#define AHB2PERIPH_BASEADDR 0x50000000UL

/* Base addresses of peripherals which are hanging on AHB1 bus */
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)

/* Base addresses of peripherals which are hanging on APB1 bus */
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASEADDR + 0x5000)

/* Base addresses of peripherals which are hanging on APB2 bus */
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR (APB2PERIPH_BASEADDR + 0x3400)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)

/* IRQ(Interrupt Request) Numbers of MCU */

#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40
#define IRQ_NO_SPI1 35
#define IRQ_NO_SPI2 36
#define IRQ_NO_SPI3 51

/****************************************************************************/
/**************** Bit position definitions of SPI peripheral ****************/
/****************************************************************************/

/* Bit position definition SPI_CR1 (Control register 1) */

#define SPI_CR1_CPHA 0		/* Clock phase */
#define SPI_CR1_CPOL 1		/* Clock polarity */
#define SPI_CR1_MSTR 2		/* Master selection */
#define SPI_CR1_BR 3		/* Baud rate */
#define SPI_CR1_SPE 6		/* SPI enable */
#define SPI_CR1_LSBFIRST 7	/* Frame format */
#define SPI_CR1_SSI 8		/* Internal slave select */
#define SPI_CR1_SSM 9		/* Software slave management */
#define SPI_CR1_RXONLY 10	/* Receive only */
#define SPI_CR1_DFF 11		/* Data frame format */
#define SPI_CR1_CRCNEXT 12	/* CRC transfer next */
#define SPI_CR1_CRCEN 13	/* Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE 14	/* Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE 15 /* Bidirectional data mode enable */

/* Bit position definition SPI_CR2 (Control register 2) */

#define SPI_CR2_RXDMAEN 0 /* RX buffer DMA enable */
#define SPI_CR2_TXDMAEN 1 /* TX buffer DMA enable */
#define SPI_CR2_SSOE 2	  /* SS output enable */
#define SPI_CR2_FRF 4	  /* Frame format */
#define SPI_CR2_ERRIE 5	  /* Error interrupt enable */
#define SPI_CR2_RXNEIE 6  /* RX buffer not empty interrupt enable */
#define SPI_CR2_TXEIE 7	  /* TX buffer empty interrput enable */

/* Bit position definition SPI_SR (Status register) */

#define SPI_SR_RXNE 0	/* Receive buffer not empty */
#define SPI_SR_TXE 1	/* Transmit buffer empty */
#define SPI_SR_CHSIDE 2 /* Channel side */
#define SPI_SR_UDR 3	/* Underrun flag */
#define SPI_SR_CRCERR 4 /* CRC error flag */
#define SPI_SR_MODF 5	/* Mode fault */
#define SPI_SR_OVR 6	/* Overrun flag */
#define SPI_SR_BSY 7	/* Busy flag */
#define SPI_SR_FRE 8	/* Frame format error */

/********************************************* Peripheral register definition structures ********************************************
 * 											 For register addresses check referance manual											*
 *************************************************************************************************************************************/

/* GPIO register definition */
typedef struct
{
	__vo uint32_t MODER;   /* GPIO port mode register (GPIOx_MODER) */
	__vo uint32_t OTYPER;  /* GPIO port output type register (GPIOx_OTYPER */
	__vo uint32_t OSPEEDR; /* GPIO port output speed register (GPIOx_OSPEEDR) */
	__vo uint32_t PUPDR;   /* GPIO port pull-up/pull-down register (GPIOx_PUPDR */
	__vo uint32_t IDR;	   /* GPIO port input data register (GPIOx_IDR) */
	__vo uint32_t ODR;	   /* GPIO port output data register (GPIOx_ODR) */
	__vo uint32_t BSSR;	   /* GPIO port bit set/reset register (GPIOx_BSRR */
	__vo uint32_t LCKR;	   /* GPIO port configuration lock register (GPIOx_LCKR) */
	__vo uint32_t AFRL;	   /* GPIO alternate function low register (GPIOx_AFRL) */
	__vo uint32_t AFRH;	   /* GPIO alternate function high register (GPIOx_AFRH) */
} GPIO_RegDef_t;

/* RCC register definiton */
typedef struct
{
	__vo uint32_t CR;		/* RCC clock control register (RCC_CR) */
	__vo uint32_t PLLCFGR;	/* RCC PLL configuration register (RCC_PLLCFGR) */
	__vo uint32_t CFGR;		/* RCC clock configuration register (RCC_CFGR) */
	__vo uint32_t CIR;		/* RCC clock interrupt register (RCC_CIR) */
	__vo uint32_t AHB1RSTR; /* RCC AHB1 peripheral reset register (RCC_AHB1RSTR) */
	__vo uint32_t AHB2RSTR; /* RCC AHB2 peripheral reset register (RCC_AHB2RSTR) */
	__vo uint32_t AHB3RSTR; /* RCC AHB3 peripheral reset register (RCC_AHB3RSTR) */
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR; /* RCC APB1 peripheral reset register (RCC_APB1RSTR) */
	__vo uint32_t APB2RSTR; /* RCC APB2 peripheral reset register (RCC_APB2RSTR) */
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR; /* RCC AHB1 peripheral clock register (RCC_AHB1ENR) */
	__vo uint32_t AHB2ENR; /* RCC AHB2 peripheral clock enable register (RCC_AHB2ENR) */
	__vo uint32_t AHB3ENR; /* RCC AHB3 peripheral clock enable register (RCC_AHB3ENR) */
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR; /* RCC APB1 peripheral clock enable register (RCC_APB1ENR) */
	__vo uint32_t APB2ENR; /* RCC APB2 peripheral clock enable register (RCC_APB2ENR) */
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR; /* RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR) */
	__vo uint32_t AHB2LPENR; /* RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR) */
	__vo uint32_t AHB3LPENR; /* RCC AHB3 peripheral clock enable in low power mode register (RCC_AHB3LPENR) */
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR; /* RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR) */
	__vo uint32_t APB2LPENR; /* RCC APB2 peripheral clock enabled in low power mode register (RCC_APB2LPENR) */
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR; /* RCC Backup domain control register (RCC_BDCR) */
	__vo uint32_t CSR;	/* RCC clock control & status register (RCC_CSR) */
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;	  /* RCC spread spectrum clock generation register (RCC_SSCGR) */
	__vo uint32_t PLLI2SCFGR; /* RCC PLLI2S configuration register (RCC_PLLI2SCFGR) */
	__vo uint32_t PLLSAICFGR; /* RCC PLL configuration register (RCC_PLLSAICFGR) */
	__vo uint32_t DCKCFGR;	  /* RCC Dedicated Clock Configuration Register (RCC_DCKCFGR) */
} RCC_RegDef_t;

/* EXTI register definition */
typedef struct
{
	__vo uint32_t IMR;	 /* Interrupt mask register (EXTI_IMR) */
	__vo uint32_t EMR;	 /* Event mask register (EXTI_EMR) */
	__vo uint32_t RTSR;	 /* Rising trigger selection register (EXTI_RTSR) */
	__vo uint32_t FTSR;	 /* Falling trigger selection register (EXTI_FTSR) */
	__vo uint32_t SWIER; /* Software interrupt event register (EXTI_SWIER) */
	__vo uint32_t PR;	 /* Pending register (EXTI_PR) */
} EXTI_RegDef_t;

/* SYSCFG (System Configuration) register definition */
typedef struct
{
	__vo uint32_t MEMRMP;  /* SYSCFG memory remap register (SYSCFG_MEMRMP) */
	__vo uint32_t PMC;	   /* SYSCFG peripheral mode configuration register (SYSCFG_PMC) */
	__vo uint32_t EXTICR1; /* SYSCFG external interrupt configuration register 1 (SYSCFG_EXTICR1) */
	__vo uint32_t EXTICR2; /* SYSCFG external interrupt configuration register 2 (SYSCFG_EXTICR2) */
	__vo uint32_t EXTICR3; /* SYSCFG external interrupt configuration register 3 (SYSCFG_EXTICR3) */
	__vo uint32_t EXTICR4; /* SYSCFG external interrupt configuration register 4 (SYSCFG_EXTICR4) */
	uint32_t RESERVED1[2]; /* 0x16-0x18 */
	__vo uint32_t CMPCR;   /* Compensation cell control register (SYSCFG_CMPCR) */
} SYSCFG_RegDef_t;

/* SPI (Serial Peripheral Interface) register definiton */
typedef struct
{
	__vo uint32_t CR1;	   /* SPI control register 1 (SPI_CR1) (not used in I2S mode) */
	__vo uint32_t CR2;	   /* SPI control register 2 (SPI_CR2) */
	__vo uint32_t SR;	   /* SPI status register (SPI_SR) */
	__vo uint32_t DR;	   /* SPI data register (SPI_DR) */
	__vo uint32_t CRCPR;   /* SPI CRC polynomial register (SPI_CRCPR) (not used in I2S mode) */
	__vo uint32_t RXCRCR;  /* SPI RX CRC register (SPI_RXCRCR) (not used in I2S mode) */
	__vo uint32_t TXCRCR;  /* SPI TX CRC register (SPI_TXCRCR) (not used in I2S mode) */
	__vo uint32_t I2SCFGR; /* SPI_I2S configuration register (SPI_I2SCFGR) */
	__vo uint32_t I2SPR;   /* SPI_I2S prescaler register (SPI_I2SPR) */
} SPI_RegDef_t;

/* Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t) */

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)

/********************************************* Peripheral clock enable/disable macros ***********************************************
 * 											 For register addresses check referance manual											*
 *************************************************************************************************************************************/

/* Clock enable macros for GPIOx peripherals */

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))

/* Clock disable macros for GPIOx peripherals */

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))

/* Clock enable macros for I2Cx peripherals */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/* Clock disable macros for I2Cx peripherals */

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

/* Clock enable macros for SPIx peripherals */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

/* Clock disable macros for SPIx peripherals */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))

/* Clock enable macros for USARTx/UARTx peripherals */

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB2ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB2ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB2ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB2ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

/* Clock disable macros for USARTx/UARTx peripherals */

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB2ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB2ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB2ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

/* Clock enable macros for SYSCFG peripherals */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/* Clock disable macros for SYSCFG peripherals */

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/********************************************* Peripheral clock register reset macros ***********************************************
 * 											 For register addresses check referance manual											*
 *************************************************************************************************************************************/

/************** GPIOx reset peripherals macros **************/

#define GPIOA_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 0));  \
		(RCC->AHB1RSTR &= ~(1 << 0)); \
	} while (0)

#define GPIOB_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 1));  \
		(RCC->AHB1RSTR &= ~(1 << 1)); \
	} while (0)

#define GPIOC_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 2));  \
		(RCC->AHB1RSTR &= ~(1 << 2)); \
	} while (0)

#define GPIOD_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 3));  \
		(RCC->AHB1RSTR &= ~(1 << 3)); \
	} while (0)

#define GPIOE_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 4));  \
		(RCC->AHB1RSTR &= ~(1 << 4)); \
	} while (0)

#define GPIOF_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 5));  \
		(RCC->AHB1RSTR &= ~(1 << 5)); \
	} while (0)

#define GPIOG_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 6));  \
		(RCC->AHB1RSTR &= ~(1 << 6)); \
	} while (0)

#define GPIOH_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 7));  \
		(RCC->AHB1RSTR &= ~(1 << 7)); \
	} while (0)

#define GPIOI_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 8));  \
		(RCC->AHB1RSTR &= ~(1 << 8)); \
	} while (0)

/************** SPIx reset peripherals macros **************/

#define SPI1_REG_RESET()              \
	do                                \
	{                                 \
		(RCC->APB2RSTR |= (1 << 12)); \
		(RCC->APB2RSTR &= (1 << 12)); \
	} while (0)

#define SPI2_REG_RESET()              \
	do                                \
	{                                 \
		(RCC->APB1RSTR |= (1 << 14)); \
		(RCC->APB1RSTR &= (1 << 14)); \
	} while (0)

#define SPI3_REG_RESET()              \
	do                                \
	{                                 \
		(RCC->APB1RSTR |= (1 << 15)); \
		(RCC->APB1RSTR &= (1 << 15)); \
	} while (0)

#define SPI4_REG_RESET()              \
	do                                \
	{                                 \
		(RCC->APB2RSTR |= (1 << 13)); \
		(RCC->APB2RSTR &= (1 << 13)); \
	} while (0)

#define GPIO_BASEADDR_TO_CODE(GPIOx) ((GPIOx == GPIOA) ? 0 : (GPIOx == GPIOB) ? 1 \
														 : (GPIOx == GPIOC)	  ? 2 \
														 : (GPIOx == GPIOD)	  ? 3 \
														 : (GPIOx == GPIOE)	  ? 4 \
														 : (GPIOx == GPIOF)	  ? 5 \
														 : (GPIOx == GPIOG)	  ? 6 \
														 : (GPIOx == GPIOH)	  ? 7 \
																			  : 0)

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
