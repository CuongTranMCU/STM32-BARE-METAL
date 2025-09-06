/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Aug 19, 2025
 *      Author: dell
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

/**
 * @brief GPIO Pin Configuration Structure
 */
typedef struct
{
    uint8_t GPIO_PinNumber;      /*!< Specifies the GPIO pin number */
    uint8_t GPIO_PinMode;        /*!< Specifies the operating mode for the selected pin */
    uint8_t GPIO_PinSpeed;       /*!< Specifies the speed for the selected pin */
    uint8_t GPIO_PinPuPdControl; /*!< Specifies the Pull-up or Pull-down activation for the selected pin */
    uint8_t GPIO_PinOPType;      /*!< Specifies the output type for the selected pin */
    uint8_t GPIO_PinAltFunMode;  /*!< Specifies the alternate function mode for the selected pin */
} GPIO_PinConfig_t;

/**
 * @brief GPIO Pin Number
 */
typedef enum
{
    GPIO_PIN_NO_0 = 0,
    GPIO_PIN_NO_1,
    GPIO_PIN_NO_2,
    GPIO_PIN_NO_3,
    GPIO_PIN_NO_4,
    GPIO_PIN_NO_5,
    GPIO_PIN_NO_6,
    GPIO_PIN_NO_7,
    GPIO_PIN_NO_8,
    GPIO_PIN_NO_9,
    GPIO_PIN_NO_10,
    GPIO_PIN_NO_11,
    GPIO_PIN_NO_12,
    GPIO_PIN_NO_13,
    GPIO_PIN_NO_14,
    GPIO_PIN_NO_15
} GPIO_PInNumber_t;
/**
 * @brief GPIO Configuration Structure
 * 
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;        /*!< Base address of the GPIO port */
    GPIO_PinConfig_t GPIO_PinConfig; /*!< GPIO pin configuration settings */
} GPIO_Handle_t;

/**
 * @brief GPIO Mode
 */
typedef enum
{
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTFN,
    GPIO_MODE_ANALOG,
    GPIO_MODE_IT_FT, /* Interrupt: Input falling edge trigger*/
    GPIO_MODE_IT_RT, /* Interrupt: Input rising edge trigger*/
    GPIO_MODE_IT_RFT /* Interrupt: Input rising falling edge trigger*/

} GPIO_Mode_t;

/**
 * @brief GPIO output type
 */
typedef enum
{
    GPIO_OP_TYPE_PP = 0,
    GPIO_OP_TYPE_OD
} GPIO_OutputType_t;

/**
 * @brief GPIO output speed
 */
typedef enum
{
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_FAST,
    GPIO_SPEED_HIGH
}GPIO_Speed_t;

/**
 * @brief GPIO pull up pull down
 */
typedef enum
{
    GPIO_NO_PUPD = 0,
    GPIO_PIN_PU,
    GPIO_PIN_PD
} GPIO_PUPD_t;
/***************************************************************************************************************************
 * @brief GPIO Driver APIs
 ***************************************************************************************************************************/
/**
 * @brief Initializes the GPIO peripheral
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @brief De-initializes the GPIO peripheral
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Clock setting for GPIO peripheral
 * 
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/**
 * @brief Reads the value from a GPIO pin
 * 
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);

/**
 * @brief Reads the value from a GPIO port
 * 
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx);

/**
 * @brief Writes a value to a GPIO pin
 * 
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t Value);

/**
 * @brief Writes a value to a GPIO port
 * 
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *GPIOx, uint16_t Value);

/**
 * @brief Toggles the state of a GPIO pin
 * 
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);

/**
 * @brief Configures the GPIO pin interrupt
 * 
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
/**
 * @brief Handles the GPIO pin interrupt
 * 
 */
void GPIO_IRQHandling(uint8_t PinNumber);
#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
