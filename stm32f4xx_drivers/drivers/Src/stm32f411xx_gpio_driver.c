/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Aug 19, 2025
 *      Author: dell
 */

#include "stm32f411xx_gpio_driver.h"

/**
 * @brief Initializes the GPIO peripheral
 * 
 * @param[in] pGPIOx        
 * 
 * @return                  void.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    /* Configure mode */

    /* Enable the peripheral clock */
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    /* Bit position of pin number gpio reg */
    uint32_t BitPosition = 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    if (GPIO_MODE_IT_FT > pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
    {
        /* Firstly clear */
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << BitPosition);
        pGPIOHandle->pGPIOx->MODER |= pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << BitPosition;
    }
    else 
    {
        /* interrupt mode */
        /* Rising trigger */
        if (GPIO_MODE_IT_RT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
        {
            /* Configure enable EXTI_RTSR Register */
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            /* Disable EXTI_FTSR */
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (GPIO_MODE_IT_FT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
        {
            /* Configure enable EXTI_FTSR Register*/
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            /* Disable EXTI_RTSR */
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (GPIO_MODE_IT_RFT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
        {
            /* Configure enable EXT_RTSR and EXTI_RTSR Register */
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        /* Configure SYSCFG_EXTICR1->4 to set EXTI line */
        SYSCFG_PCLK_EN();
        /* Choose EXTICR1-4 */
        uint32_t EXTICRx = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        /* Choose bit position */
        uint32_t EXTICRxBitPosition = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
        /* Configure EXTICR port code*/
        uint32_t EXTICRxPortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        /* Firstly clear */
        SYSCFG->EXTICR[EXTICRx] &= ~(0xF << EXTICRxBitPosition);
        SYSCFG->EXTICR[EXTICRx] |= EXTICRxPortCode << EXTICRxBitPosition;

        /* Enable delevery interrupt from peri -> cpu: IMR -> line is not masked -> allow to delivery to NVIC */
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    /* Configure speed */
    /* Firstly clear */
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << BitPosition);
    pGPIOHandle->pGPIOx->OSPEEDR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << BitPosition;


    /* Configure pull up pull down */
    /* Firstly clear */
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << BitPosition);
    pGPIOHandle->pGPIOx->PUPDR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << BitPosition;
    
    /* Configure output type*/
    /* Firstly clear */
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << (BitPosition/2));
    pGPIOHandle->pGPIOx->OTYPER |= pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (BitPosition / 2);

    /* Configure alt func */
    if (GPIO_MODE_ALTFN == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
    {
        /* Choose low or high */
        uint32_t AltFuncIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        /* Choose bit position */
        uint32_t AltFuncBitPosition = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
        /* Configure alt func reg */
        /* Firstly clear */
        pGPIOHandle->pGPIOx->AFR[AltFuncIndex] &= ~(0xFF << AltFuncBitPosition);
        pGPIOHandle->pGPIOx->AFR[AltFuncIndex] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << AltFuncBitPosition;
    }
}

/**
 * @brief De-initializes the GPIO peripheral
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (GPIOA == pGPIOx)
    {
        GPIOA_REG_RESET();
    }
    else if (GPIOB == pGPIOx)
    {
        GPIOB_REG_RESET();
    }
    else if (GPIOC == pGPIOx)
    {
        GPIOC_REG_RESET();
    }
    else if (GPIOD == pGPIOx)
    {
        GPIOD_REG_RESET();
    }
    else if (GPIOE == pGPIOx)
    {
        GPIOE_REG_RESET();
    }
    else if (GPIOH == pGPIOx)
    {
        GPIOH_REG_RESET();
    }
}

/**
 * @brief Clock setting for GPIO peripheral
 * 
 * @param[in] pGPIOx
 * @param[in] EnorDi
 * 
 * @return                  void.
 * 
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (ENABLE == EnorDi)
    {
        if (GPIOA == pGPIOx)
        {
            GPIOA_PCLK_EN();
        }
        else if (GPIOB == pGPIOx)
        {
            GPIOB_PCLK_EN();
        }
        else if (GPIOC == pGPIOx)
        {
            GPIOC_PCLK_EN();
        }
        else if (GPIOD == pGPIOx)
        {
            GPIOD_PCLK_EN();
        }
        else if (GPIOE == pGPIOx)
        {
            GPIOE_PCLK_EN();
        }
        else if (GPIOH == pGPIOx)
        {
            GPIOH_PCLK_EN();
        }
    }
    else
    {
        if (GPIOA == pGPIOx)
        {
            GPIOA_PCLK_DI();
        }
        else if (GPIOB == pGPIOx)
        {
            GPIOB_PCLK_DI();
        }
        else if (GPIOC == pGPIOx)
        {
            GPIOC_PCLK_DI();
        }
        else if (GPIOD == pGPIOx)
        {
            GPIOD_PCLK_DI();
        }
        else if (GPIOE == pGPIOx)
        {
            GPIOE_PCLK_DI();
        }
        else if (GPIOH == pGPIOx)
        {
            GPIOH_PCLK_DI();
        } 
    }
}

/**
 * @brief Reads the value from a GPIO pin
 * 
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber)
{
    uint8_t InputData = (uint8_t)((GPIOx->IDR >> PinNumber) & 0x01);
    return InputData;
}

/**
 * @brief Reads the value from a GPIO port
 * 
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx)
{
    return (uint16_t)(GPIOx->IDR);
}

/**
 * @brief Writes a value to a GPIO pin
 * 
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (GPIO_PIN_SET == Value)
    {
        GPIOx->ODR |= (GPIO_PIN_SET << PinNumber);
    }
    else 
    {
        GPIOx->ODR &= ~(GPIO_PIN_SET << PinNumber);
    }
}

/**
 * @brief Writes a value to a GPIO port
 * 
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *GPIOx, uint16_t Value)
{
    GPIOx->ODR = Value;
}

/**
 * @brief Toggles the state of a GPIO pin
 * 
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber)
{
    GPIOx->ODR ^= (GPIO_PIN_SET << PinNumber);
}

/**
 * @brief Configures the GPIO pin interrupt
 * 
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    /* Enable, disable interupt */
    if (ENABLE == EnorDi)
    {
        if (IRQNumber < 32)
        {
            /* Configure NVIC_ISRER0 */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 32 && IRQNumber < 64)
        {
            /* Configure NVIC_ISRER1 */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Configure NVIC_ISRER2 */
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
        }
    }
    else
    {
        if (IRQNumber < 32)
        {
            /* Configure NVIC_ICER0 */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 32 && IRQNumber < 64)
        {
            /* Configure NVIC_ICER1 */
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Configure NVIC_ICER2 */
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }      
    }

}
/**
 * @brief Configures the interrupt priority
 * 
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    /* Find IPRx */
    uint8_t IPRx = IRQNumber / 4;
    /* IPRx bit position */
    uint8_t IPRxBitPosition = (8 * (IRQNumber % 4)) + (8 - NO_PR_BITS_IMPLEMENTED);
    /* Configure IRQ Priority */
    *(NVIC_PR_BASE_ADDR + (IPRx * 4)) |= (IRQPriority << IPRxBitPosition);
}
/**
 * @brief Handles the GPIO pin interrupt
 * 
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    /* Check pending set in EXTI */
    if (EXTI->PR & (1 << PinNumber))
    {
        /* Write 1 to clear */
        EXTI->PR |= (1 < PinNumber);
    }
}