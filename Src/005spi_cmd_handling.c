/*
 * 005spi_cmd_handling.c
 *
 *  Created on: Aug 9, 2023
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

extern void initialise_monitor_handles();

#define COMMAND_LED_CTRL 0x50
#define COMMAND_SENSOR_READ 0x51
#define COMMAND_LED_READ 0x52
#define COMMAND_PRINT 0x53
#define COMMAND_ID_READ 0x54

#define LED_ON 1
#define LED_OFF 0

#define ANALOG_PIN0 0
#define ANALOG_PIN1 1
#define ANALOG_PIN2 2
#define ANALOG_PIN3 3
#define ANALOG_PIN4 4
#define ANALOG_PIN5 5

#define LED_PIN 9

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

    SPI_Handle_t SPI2handle;

    SPI2handle.pSPIx = SPI2;
    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2handle.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV8; // generates sclk of 2MHz
    SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin

    SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOBtn;

    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);
}

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++)
        ;
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
    if (ackbyte == 0xF5)
    {
        return 1;
    }
    return 0;
}

int main(void)
{

    uint8_t dummy_write = 0xff;
    uint8_t dummy_read;

    initialise_monitor_handles();

    GPIO_ButtonInit();

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

    GPIO_Handle_t GPIOLed;
    GPIOLed.pGPIOx = GPIOD;
    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOLed);

    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&GPIOLed);

    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&GPIOLed);

    printf("Everything initlialized!\n");

    while (1)
    {
        // wait till button is pressed
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
            ;

        GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIO_PIN_NO_15);
        GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIO_PIN_NO_13);

        // to avoid button de-bouncing related issues 200ms of delay
        delay();

        // enable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, ENABLE);

        // 1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

        uint8_t commandcode = COMMAND_LED_CTRL;
        uint8_t ackbyte;
        uint8_t args[2];

        // send command
        SPI_SendData(SPI2, &commandcode, 1);

        // do dummy read to clear off the RXNE
        SPI_ReceiveData(SPI2, &dummy_read, 1);

        // Send some dummy bits (1 byte) fetch the response from the slave
        SPI_SendData(SPI2, &dummy_write, 1);

        // read the ack byte received
        SPI_ReceiveData(SPI2, &ackbyte, 1);

        if (SPI_VerifyResponse(ackbyte))
        {
            args[0] = LED_PIN;
            args[1] = LED_ON;

            // send arguments
            SPI_SendData(SPI2, args, 2);
            GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIO_PIN_NO_13);
            // dummy read
            SPI_ReceiveData(SPI2, &dummy_read, 1);
            GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIO_PIN_NO_15);
            printf("COMMAND_LED_CTRL initialized.\n");
        }

        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
            ;

        GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIO_PIN_NO_12);

        // to avoid button de-bouncing related issues 200ms of delay
        delay();

        commandcode = COMMAND_SENSOR_READ;

        // send command
        SPI_SendData(SPI2, &commandcode, 1);

        // do dummy read to clear off the RXNE
        SPI_ReceiveData(SPI2, &dummy_read, 1);

        // Send some dummy bits (1 byte) fetch the response from the slave
        SPI_SendData(SPI2, &dummy_write, 1);

        // read the ack byte received
        SPI_ReceiveData(SPI2, &ackbyte, 1);

        if (SPI_VerifyResponse(ackbyte))
        {
            args[0] = ANALOG_PIN0;

            // send arguments
            SPI_SendData(SPI2, &args[0], 1);
            // dummy read
            SPI_ReceiveData(SPI2, &dummy_read, 1);

            delay();

            // Send some dummy bits (1 byte) fetch the response from the slave
            SPI_SendData(SPI2, &dummy_write, 1);

            uint8_t analog_read;
            SPI_ReceiveData(SPI2, &analog_read, 1);
            printf("COMMAND_SENSOR_READ: %d\n", analog_read);

            if (analog_read == 0)
            {
                GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIO_PIN_NO_12);
            }
        }

        // lets confirm SPI is not busy
        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG))
            ;

        // Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, DISABLE);
        printf("SPI communication closed.\n");
    }

    return 0;
}
