/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 3, 2023
 *  Author: Taha Eren Karakış
 */

#include "stm32f407xx_gpio_driver.h"

/************************* Peripheral Clock Setup *************************/

/***************************************************************
 * @fn				GPIO_PeriClockControl
 *
 * @brief			This function enables or disables peripheral clock for the given GPIO port
 *
 * @param pGPIOx	base address of the gpio peripheral
 * @param EnOrDi	ENABLE or DISABLE macros
 *
 * @returns			none
 *
 * @note			none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else if (EnOrDi == DISABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/************************* Init and De-Init *************************/

/***************************************************************
 * @fn					GPIO_Init
 *
 * @brief				This function initializes the GPIO
 *
 * @param pGPIOHandle	base address of the gpio peripheral
 *
 * @returns				none
 *
 * @note				none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	/* Enable the GPIO clock */
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// Configuring the mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode > GPIO_MODE_ANALOG && pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
	{
		// Interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the Falling Trigger Selection Register (FTSR)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the RTST bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the Rising Trigger Selection Register (RTSR)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the RTST bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Configure the Rising-Falling Trigger Selection Register (RTSR-FTSR)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR (System Configuration External Interrupt Config. Register)
		SYSCFG_PCLK_EN();
		uint8_t EXTIReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t EXTIPort = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		switch (EXTIReg)
		{
		case 0:
			SYSCFG->EXTICR1 = PortCode << (EXTIPort * 4);
			break;
		case 1:
			SYSCFG->EXTICR2 = PortCode << (EXTIPort * 4);
			break;
		case 2:
			SYSCFG->EXTICR3 = PortCode << (EXTIPort * 4);
			break;
		case 3:
			SYSCFG->EXTICR4 = PortCode << (EXTIPort * 4);
			break;
		}

		// Enable EXTI interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// Configuring the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// Configuring the pull-up pull-down res
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// Configuring the output type
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp = 0;
	}

	// Configuring the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t AFR_Location;
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
		{
			AFR_Location = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * AFR_Location));
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 7 && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 15)
		{
			AFR_Location = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * AFR_Location));
		}
	}
}

/***************************************************************
 * @fn					GPIO_DeInit
 *
 * @brief				This function de-initializes the GPIO
 *
 * @param pGPIOx		base address of the gpio peripheral
 *
 * @returns				none
 *
 * @note				none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/************************* Data read and write *************************/

/***************************************************************
 * @fn					GPIO_ReadFromInputPin
 *
 * @brief				This function reads the data from given GPIO pin
 *
 * @param pGPIOx		base address of the gpio peripheral
 * @param PinNumber		input pin number
 *
 * @returns				Data read
 *
 * @note				none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

/***************************************************************
 * @fn					GPIO_ReadFromInputPort
 *
 * @brief				This function reads the data from given GPIO port
 *
 * @param pGPIOx		base address of the gpio peripheral
 *
 * @returns				Data read
 *
 * @note				none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint8_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/***************************************************************
 * @fn					GPIO_WriteToOutputPin
 *
 * @brief				This function writes to the given GPIO pin
 *
 * @param pGPIOx		base address of the gpio peripheral
 * @param PinNumber		output pin number
 * @param Value			output pin value
 *
 * @returns				none
 *
 * @note				none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else if (Value == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***************************************************************
 * @fn					GPIO_WriteToOutputPort
 *
 * @brief				This function writes to the given GPIO port
 *
 * @param pGPIOx		base address of the gpio peripheral
 * @param Value			output pin value
 *
 * @returns				none
 *
 * @note				none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***************************************************************
 * @fn					GPIO_ToggleOutputPin
 *
 * @brief				This function write to the given GPIO port
 *
 * @param pGPIOx		base address of the gpio peripheral
 * @param PinNumber		output pin number
 *
 * @return				none
 *
 * @note				none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/************************* IRQ Configuration and ISER handling *************************/
/*
 * ISER -> Interrupt set enable register
 * ICER -> Interrupt clear enable register
 */

/***************************************************************
 * @fn					GPIO_IRQInterruptConfig
 *
 * @brief				This function configures the interrupt
 *
 * @param IRQNumber		interrupt number
 * @param EnOrDi		ENABLE or DISABLE macros
 *
 * @return				none
 *
 * @note				none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			/* Program ISER0 register */
			*NVIC_ISER0_PTR |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			/* Program ISER1 register */
			*NVIC_ISER1_PTR |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			/* Program ISER2 register */
			*NVIC_ISER2_PTR |= (1 << (IRQNumber % 64));
		}
	}
	else if (EnOrDi == DISABLE)
	{
		if (IRQNumber <= 31)
		{
			/* Program ICER0 register */
			*NVIC_ICER0_PTR |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			/* Program ICER1 register */
			*NVIC_ICER1_PTR |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			/* Program ICER2 register */
			*NVIC_ICER2_PTR |= (1 << (IRQNumber % 64));
		}
	}
}

/***************************************************************
 * @fn					GPIO_IRQPriorityConfig
 *
 * @brief				This function configures the interrupt request priority
 *
 * @param IRQPriority	interrupt priority
 *
 * @return				none
 *
 * @note				none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	/* Finding the Input Priority Register(IPR) */
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRxSection = IRQNumber % 4;

	uint8_t shift_amount = (8 * IPRxSection) + (8 - NVIC_NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR_PTR + (IPRx * 4)) |= (IRQPriority << shift_amount);
}

/***************************************************************
 * @fn					GPIO_IRQHandling
 *
 * @brief				This function handles the interrupt for given pin
 *
 * @param PinNumber		pin number for handling interrupt
 *
 * @return				none
 *
 * @note				none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		// Clear pending bit
		EXTI->PR |= (1 << PinNumber);
	}
}
