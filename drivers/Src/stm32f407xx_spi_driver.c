/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 7, 2023
 *      Author: jpant
 */

#include "stm32f407xx_spi_driver.h"

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
 * @note				none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    /* Configuring SPI_CR1 register */
    uint32_t temp = 0;

    /* Configuring device mode */
    temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

    /* Configuring bus config */
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        /* Bi-directional mode should be cleared */
        temp &= ~(1 << 15);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        /* Bi-directional mode should be set */
        temp |= (1 << 15);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        /* Bi-directional mode should be cleared */
        temp &= ~(1 << 15);
        /* RXONLY must be set */
        temp |= (1 << 10);
    }

    /* Configring the SPI serial clock speed (baud rate) */
    temp |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << 3;

    /* Configuring the DFF */
    temp |= pSPIHandle->SPIConfig.SPI_DFF << 11;

    /* Configuring the CPOL */
    temp |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

    /* Configuring the CPHA */
    temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    pSPIHandle->pSPIx->CR1 = temp;
}

/***************************************************************
 * @fn					SPI_DeInit
 *
 * @brief				This function de-initializes the SPI
 *
 * @param pSPIx		base address of the spi peripheral
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