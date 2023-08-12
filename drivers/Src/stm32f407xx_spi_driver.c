/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 7, 2023
 *      Author: jpant
 */

#include "stm32f407xx_spi_driver.h"
#include <stddef.h>

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/***************************************************************
 * @fn				SPI_PeriClockControl
 *
 * @brief			This function enables or disables peripheral clock for the given SPI port
 *
 * @param pSPIx	    base address of the spi peripheral
 * @param EnOrDi	ENABLE or DISABLE macros
 *
 * @returns			none
 *
 * @note			none
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else if (EnOrDi == DISABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}

/***************************************************************
 * @fn					SPI_Init
 *
 * @brief				This function initializes the SPI
 *
 * @param pSPIHandle	base address of the spi peripheral
 *
 * @returns				none
 *
 * @note				This function doesn't enable the SPI. Use SPI_PeripheralControl function to enable
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    /* Configuring SPI_CR1 register */
    uint32_t temp = 0;

    /* Enable the peripheral clock */
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    /* Configuring device mode */
    temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    /* Configuring bus config */
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        /* Bi-directional mode should be cleared */
        temp &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        /* Bi-directional mode should be set */
        temp |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        /* Bi-directional mode should be cleared */
        temp &= ~(1 << SPI_CR1_BIDIMODE);
        /* RXONLY must be set */
        temp |= (1 << SPI_CR1_RXONLY);
    }

    /* Configring the SPI serial clock speed (baud rate) */
    temp |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR;

    /* Configuring the DFF */
    temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    /* Configuring the CPOL */
    temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    /* Configuring the CPHA */
    temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    pSPIHandle->pSPIx->CR1 = temp;
}

/***************************************************************
 * @fn					SPI_DeInit
 *
 * @brief				This function de-initializes the SPI
 *
 * @param pSPIx		    base address of the spi peripheral
 *
 * @returns				none
 *
 * @note				none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
    else if (pSPIx == SPI4)
    {
        SPI4_REG_RESET();
    }
}

/***************************************************************
 * @fn					SPI_GetFlagStatus
 *
 * @brief				This function checks the SPI flag state
 *
 * @param pSPIx		    base address of the spi peripheral
 * @param FlagName      flag to check
 *
 * @returns				Flag state
 *
 * @note				none
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{

    if (pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }

    return FLAG_RESET;
}

/***************************************************************
 * @fn					SPI_SendData
 *
 * @brief				This function de-initializes the SPI
 *
 * @param pSPIx		    base address of the spi peripheral
 * @param pTxBuffer
 * @param Len           number of bytes to transmit
 *
 * @returns				none
 *
 * @note				This is a blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        /* Wait until TXE set */
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
            ;

        /* Check the DFF bit in CR1 */
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            /* 16 bit DFF */
            /* Load the data in to the DR (Data register) */
            pSPIx->DR = *((uint16_t *)pTxBuffer);
            Len -= 2;
            (uint16_t *)pTxBuffer++;
        }
        else if (!(pSPIx->CR1 & (1 << SPI_CR1_DFF)))
        {
            /* 8 bit DFF */
            /* Load the data in to the DR (Data register) */
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}

/***************************************************************
 * @fn					SPI_ReceiveData
 *
 * @brief				This function de-initializes the SPI
 *
 * @param pSPIx		    base address of the spi peripheral
 * @param pTxBuffer
 * @param Len           number of bytes to transmit
 *
 * @returns				none
 *
 * @note				This is a blocking call
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        /* Wait until RXNE set */
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
            ;

        /* Check the DFF bit in CR1 */
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            /* 16 bit DFF */
            /* Load the data from DR to the RXBuffer address */
            *((uint16_t *)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            (uint16_t *)pRxBuffer++;
        }
        else
        {
            /* 8 bit DFF */
            /* Load the data in to the DR (Data register) */
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

/***************************************************************
 * @fn					SPI_PeripheralControl
 *
 * @brief				This function de-initializes the SPI
 *
 * @param pSPIx		    base address of the spi peripheral
 * @param EnOrDi	    ENABLE or DISABLE macros
 *
 * @returns				none
 *
 * @note				none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else if (EnOrDi == DISABLE)
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/***************************************************************
 * @fn					SPI_SSIConfig
 *
 * @brief				This function configures the SSI
 *
 * @param pSPIx		    base address of the spi peripheral
 * @param EnOrDi	    ENABLE or DISABLE macros
 *
 * @returns				none
 *
 * @note				none
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else if (EnOrDi == DISABLE)
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

/***************************************************************
 * @fn					SPI_SSOEConfig
 *
 * @brief				This function configures the SSOE
 *
 * @param pSPIx		    base address of the spi peripheral
 * @param EnOrDi	    ENABLE or DISABLE macros
 *
 * @returns				none
 *
 * @note				none
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else if (EnOrDi == DISABLE)
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    /* Finding the Input Priority Register(IPR) */
    uint8_t IPRx = IRQNumber / 4;
    uint8_t IPRxSection = IRQNumber % 4;

    uint8_t shift_amount = (8 * IPRxSection) + (8 - NVIC_NO_PR_BITS_IMPLEMENTED);
    *(NVIC_IPR_BASEADDR_PTR + (IPRx * 4)) |= (IRQPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX)
    {
        /* Save the Tx buffer address and Len information in some global variables */
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        /* Mark the SPI state as busy in transmision so other code can't take over same SPI periph. until transmission is over */
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        /* Enable the TXEIE ( Tx buffer empty interrupt enable ) */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

        /* Data transmission will be handled by the ISR (Interrupt Service Routine) code (will implement later) */
    }
    return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX)
    {
        /* Save the Rx buffer address and Len information in some global variables */
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        /* Mark the SPI state as busy in transmision so other code can't take over same SPI periph. until transmission is over */
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        /* Enable the RXNEIE ( RX buffer not empty interrupt enable ) */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

        /* Data transmission will be handled by the ISR (Interrupt Service Routine) code (will implement later) */
    }
    return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
    uint8_t test_sr, test_cr2;

    /* Check for TXE */
    test_sr = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    test_cr2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if (test_sr && test_cr2)
    {
        /* Handle TXE */
        spi_txe_interrupt_handle(pHandle);
    }

    /* Check for RXNE */
    test_sr = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    test_cr2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if (test_sr && test_cr2)
    {
        /* Handle RXNE */
        spi_rxne_interrupt_handle(pHandle);
    }

    /* Check for OVR flag */
    test_sr = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    test_cr2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if (test_sr && test_cr2)
    {
        /* Handle OVR */
        spi_ovr_err_interrupt_handle(pHandle);
    }
}

/*****************************************************************************/
/********************** Helper function implementations **********************/
/****************************************************************************/

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    /* Check the DFF bit in CR1 */
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        /* 16 bit DFF */
        /* Load the data in to the DR (Data register) */
        pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen -= 2;
        (uint16_t *)pSPIHandle->pTxBuffer++;
    }
    else if (!(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
    {
        /* 8 bit DFF */
        /* Load the data in to the DR (Data register) */
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if (!pSPIHandle->TxLen)
    {
        /* TxLen is zero, so close the SPI communication and inform the application that Tx is over */
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    /* Check the DFF bit in CR1 */
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        /* 16 bit DFF */
        /* Load the data from DR to the RXBuffer address */
        *((uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        (uint16_t *)pSPIHandle->pRxBuffer++;
    }
    else
    {
        /* 8 bit DFF */
        /* Load the data in to the DR (Data register) */
        *pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if (!pSPIHandle->RxLen)
    {
        /* RxLen is zero, so close the SPI communication and inform the application that Rx is over */
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
    /* Clear the overrun flag */
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;
    /* Inform the application */
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
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

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    /* This is a weak implementation. The application may override this function. */
}