/*
 * 006spi_message_rcv_it.c
 *
 *  Created on: Aug 11, 2023
 *  Author: Taha Eren Karakış
 */

#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

SPI_Handle_t SPI2Handle;

#define MAX_LEN 255

char ReceivedMessageBuffer[MAX_LEN];

__vo char ReadByte;

__vo uint8_t data_finished = 0;

__vo uint8_t data_avaliable = 0;

extern void initialise_monitor_handles();

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV8; // generates sclk of 2MHz
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin

    SPI_Init(&SPI2Handle);
}

void Slave_GPIO_InterruptPinInit(void)
{
    GPIO_Handle_t PendingMessagePin;

    PendingMessagePin.pGPIOx = GPIOD;
    PendingMessagePin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    PendingMessagePin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    PendingMessagePin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    PendingMessagePin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&PendingMessagePin);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
}

int main(void)
{
    initialise_monitor_handles(); // For debugging

    uint8_t dummy_data = 0xff;

    /* Inializations */
    Slave_GPIO_InterruptPinInit();

    // this function is used to initialize the GPIO pins to behave as SPI2 pins
    SPI2_GPIOInits();

    // This function is used to initialize the SPI2 peripheral parameters
    SPI2_Inits();

    /*
     * making SSOE 1 does NSS output enable.
     * The NSS pin is automatically managed by the hardware.
     * i.e when SPE=1 , NSS will be pulled to low
     * and NSS pin will be high when SPE=0
     */
    SPI_SSOEConfig(SPI2, ENABLE);

    SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

    printf("Inializations successfull!\n");

    while (1)
    {
        data_finished = 0;

        while (!data_avaliable)
            ; /* Wait until data avaliable interrupt from transmitter device (slave) */

        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

        /* Enable the SPI2 peripheral */
        SPI_PeripheralControl(SPI2, ENABLE);

        while (!data_finished)
        {
            /* Fetch the data from the SPI peripheral byte by byte in Interrupt mode */
            while (SPI_SendDataIT(&SPI2Handle, &dummy_data, 1) == SPI_BUSY_IN_TX)
                ;
            while (SPI_ReceiveDataIT(&SPI2Handle, &ReadByte, 1) == SPI_BUSY_IN_RX)
                ;
        }

        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG))
            ;

        SPI_PeripheralControl(SPI2, DISABLE);

        printf("Received data: %s\n", ReceivedMessageBuffer);

        data_avaliable = 0;

        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
    }

    return 0;
}

/* Runs when a data byte is received from the peripheral over SPI */
void SPI2_IRQHandler(void)
{
    SPI_IRQHandling(&SPI2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    static uint32_t i = 0;

    /* In the RX complete event copy data in to ReceievedMessageBuffer */
    if (AppEv == SPI_EVENT_RX_CMPLT)
    {
        ReceivedMessageBuffer[i++] = ReadByte;
        if (ReadByte == '\0' || i == MAX_LEN)
        {
            data_finished = 1;
            ReceivedMessageBuffer[i - 1] = '\0';
            i = 0;
        }
    }
}

/* Slave data availble interrupt handler */
void EXTI9_5_IRQHandler()
{
    GPIO_IRQHandling(GPIO_PIN_NO_6);
    data_avaliable = 1;
}