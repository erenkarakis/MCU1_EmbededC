/*
 * 002LedButton.c
 *
 *  Created on: Aug 5, 2023
 *  Author: Taha Eren Karakış
 */
#include "stm32f407xx.h"

void delay(void)
{
    for (uint32_t i = 0; i < 500000; i++)
        ;
}

int main(void)
{
    GPIO_Handle_t GPIOLed, GPIOBtn;

    GPIOLed.pGPIOx = GPIOD;
    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOLed.pGPIOx, ENABLE);
    GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE);
    GPIO_Init(&GPIOLed);

    while (1)
    {
        if (GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, GPIOBtn.GPIO_PinConfig.GPIO_PinNumber) == HIGH)
        {
            GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber);
            delay();
        }
    }

    return 0;
}