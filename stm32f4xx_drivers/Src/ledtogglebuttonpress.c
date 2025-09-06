/*
 * ledtogglebuttonpress.c
 *
 *  Created on: Aug 21, 2025
 *      Author: dell
 */
#include "stm32f411xx_gpio_driver.h"
void delay()
{
    for (uint32_t i = 0; i < 200000; i++);
}
int main()
{
    /* Led green PA5*/
    GPIO_Handle_t GpioLed = {
        .pGPIOx = GPIOA,
        .GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5,
        .GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT,
        .GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP,
        .GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU,
        .GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST
    };
    GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
    GPIO_Init(&GpioLed);
    /* Button B1: PC13 */
    GPIO_Handle_t GpioButton = {
        .pGPIOx = GPIOC,
        .GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13,
        .GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT,
        .GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD,
        .GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST
    };
    GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);
    GPIO_Init(&GpioButton);
    /* Use polling */
    while (1)
    {
        if (GPIO_ReadFromInputPin(GpioButton.pGPIOx, GpioButton.GPIO_PinConfig.GPIO_PinNumber) == 0)
        {
            delay();
            GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
        }
    }
    return 0;
}


