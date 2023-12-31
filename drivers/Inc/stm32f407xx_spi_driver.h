/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Aug 7, 2023
 *  Author: Taha Eren Karakış
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
    uint8_t SPI_DeviceMode; /* SPI_DEVICE_MODE_MASTER or SPI_DEVICE_MODE_SLAVE */
    uint8_t SPI_BusConfig;
    uint8_t SPI_SCLKSpeed;
    uint8_t SPI_DFF;  /* Data frame format */
    uint8_t SPI_CPOL; /* Clock polarity */
    uint8_t SPI_CPHA; /* Clock phase */
    uint8_t SPI_SSM;  /* Software slave management */
} SPI_Config_t;

typedef struct
{
    SPI_RegDef_t *pSPIx; /* Holds the base address of SPIx (1, 2 ,3) */
    SPI_Config_t SPIConfig;
    uint8_t *pTxBuffer; /* To store the application Tx buffer address */
    uint8_t *pRxBuffer; /* To store the application Rx buffer address */
    uint32_t TxLen;     /* To store transmit buffer length */
    uint32_t RxLen;     /* To store received buffer length */
    uint8_t TxState;    /* To store Tx state */
    uint8_t RxState;    /* To store Rx state */
} SPI_Handle_t;

/****************************************************************************/
/************************* SPI Configuration Macros *************************/
/****************************************************************************/

/*
@SPI_DeviceMode
*/
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

/*
@SPI_BusConfig
*/
#define SPI_BUS_CONFIG_FD 1             /* Full duplex */
#define SPI_BUS_CONFIG_HD 2             /* Half duplex */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3 /* Simplex rx only */

/*
@SPI_SCLKSpeed
*/
#define SPI_SCLK_SPEED_DIV2 0   /* Uses the bus speed divided by 2 */
#define SPI_SCLK_SPEED_DIV4 1   /* Uses the bus speed divided by 4 */
#define SPI_SCLK_SPEED_DIV8 2   /* Uses the bus speed divided by 8 */
#define SPI_SCLK_SPEED_DIV16 3  /* Uses the bus speed divided by 16 */
#define SPI_SCLK_SPEED_DIV32 4  /* Uses the bus speed divided by 32 */
#define SPI_SCLK_SPEED_DIV64 5  /* Uses the bus speed divided by 64 */
#define SPI_SCLK_SPEED_DIV128 6 /* Uses the bus speed divided by 128 */
#define SPI_SCLK_SPEED_DIV256 7 /* Uses the bus speed divided by 256 */

/*
@SPI_DFF
*/
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

/*
@SPI_CPOL
*/
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
@SPI_CPHA
*/
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
@SPI_SSM
*/
#define SPI_SSM_EN 1 /* Software contolled (SSM enable) */
#define SPI_SSM_DI 0 /* Hardware controlled (SSM disable) */

/** Possible SPI Application States **/

#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

/** Possible SPI Application Events **/

#define SPI_EVENT_TX_CMPLT 1 /* Tx complete event */
#define SPI_EVENT_RX_CMPLT 2 /* Rx complete event */
#define SPI_EVENT_OVR_ERR 3  /* Overrun error */

/****************************************************************************/
/************************* SPI Releted Status Flags *************************/
/****************************************************************************/

#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1 << SPI_SR_BSY)

/*********************************************************************************************
 *								APIs supported by this driver
 *				For more information about the APIs check the function definitions
 *********************************************************************************************/

/************************* Peripheral Clock Setup *************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/************************* Init and De-Init *************************/

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/************************* Data Send and Receive *************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/************************* IRQ Configuration and ISR handling *************************/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/************************* Other Pripheral Control APIs *************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/************************* Application Callback *************************/

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
