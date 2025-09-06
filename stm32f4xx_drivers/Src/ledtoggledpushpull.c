
#include "stm32f411xx_gpio_driver.h"
void delay()
{
    for (uint32_t i = 0; i < 500000; i++);
}
int main()
{
    /* Led green PA5*/
    GPIO_Handle_t GpioLed = {
        .pGPIOx = GPIOA,
        .GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5,
        .GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT,
        .GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD ,
        .GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU,
        .GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST
    };
    
    GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

    GPIO_Init(&GpioLed);
    while (1)
    {
        GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
        delay();
    }
    return 0;
}


