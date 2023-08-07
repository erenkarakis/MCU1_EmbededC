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
}