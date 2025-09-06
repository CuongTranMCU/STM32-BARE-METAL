/*
 * stm32f411xx.h
 *
 *  Created on: Aug 17, 2025
 *      Author: dell
 */

#ifndef STM32F411XX_H_
#define STM32F411XX_H_

#include <stdint.h>
#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/**
 * @brief base address of FLASH and SRAM memories
 */
#define FLASH_BASEADDR				0x08000000U
#define SRAM_BASEADDR				0x20000000U
#define ROM_BASEADDR				0x1FFF0000U

/**
 * @brief AHBx and APBx Bus Peripheral base addressesS
 */
#define PERIPH_BASEADDR					  0x40000000U
#define APB1_PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR			0x40010000U
#define AHB1_PERIPH_BASEADDR			0x40020000U
#define AHB2_PERIPH_BASEADDR			0x50000000U

/**
 * @brief Base addresses of AHB1 Bus
 */
#define GPIOA_BASEADDR                   (AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					         (AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					         (AHB1_PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					         (AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR 					         (AHB1_PERIPH_BASEADDR + 0x1C00)

#define RCC_BASEADDR                     (AHB1_PERIPH_BASEADDR + 0x3800)

/**
 * @brief Base addresses of APB1 Bus
 */
#define I2C1_BASEADDR						         (APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						         (APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						         (APB1_PERIPH_BASEADDR + 0x5C00)
        
#define SPI2_BASEADDR						         (APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						         (APB1_PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						       (APB1_PERIPH_BASEADDR + 0x4400)

/**
 * @brief Base addresses of APB2
 */
#define EXTI_BASEADDR						         (APB2_PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						         (APB2_PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR                    (APB2_PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR        				   (APB2_PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						       (APB2_PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						       (APB2_PERIPH_BASEADDR + 0x1400)



/**********************************peripheral register definition structures **********************************/
/**
 * @brief GPIO
 */
typedef struct
{
	__vo uint32_t MODER;          /*!< GPIO port mode register,          Address offset: 0x00      */
	__vo uint32_t OTYPER;         /*!<      							 Address offset: 0x04      */
	__vo uint32_t OSPEEDR;        /*!<      							 Address offset: 0x08      */
	__vo uint32_t PUPDR;          /*!<      							 Address offset: 0x0C      */
	__vo uint32_t IDR;            /*!<      							 Address offset: 0x10      */
	__vo uint32_t ODR;		        /*!<      							 Address offset: 0x14      */
	__vo uint32_t BSRR;           /*!<      							 Address offset: 0x18      */
	__vo uint32_t LCKR;           /*!<      							 Address offset: 0x1C      */
	__vo uint32_t AFR[2];         /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						    Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									            Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										            Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									            Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					    Address offset: 0x14 */

}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,      Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									    Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									      Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2]; /*!< TODO          							    Reserved, 0x18-0x1C    	  */
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2]; /*!<                                Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                           Address offset: 0x2C   	  */
} SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4  				((SPI_RegDef_t*)SPI4_BASEADDR)

#define RCC 				  ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				  ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/**
 * Clock Enable Macros for GPIOx peripherials
 */
#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |= (1 << 7))

/**
 * @brief Clock Enable Macros for I2Cx peripherials
 */
#define I2C1_PCLK_EN()     (RCC->APB1ENR |= (1 << 21))

/**
 * @brief Clock Enable Macros for SPIx peripherials
 */
#define SPI1_PCLK_EN()     (RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN()     (RCC->APB2ENR |= (1 << 13))
#define SPI2_PCLK_EN()     (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()     (RCC->APB1ENR |= (1 << 15))

/**
 * @brief Clock Enable Macros for USARTx peripherials
 */
#define USART1_PCLK_EN()   (RCC->APB2ENR |= (1 << 4))


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |= (1 << 14))

/**
 * @brief Clock Disable Macros for GPIOx peripherials
 */
#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 7))

/**
 * @brief Clock Disable Macros for I2Cx peripherials
 */
#define I2C1_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 21))

/**
 * @brief Clock Disable Macros for SPIx peripherials
 */
#define SPI1_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 13))
#define SPI2_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 15))

/**
 * @brief Clock Disable Macros for USARTx peripherials
 */
#define USART1_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 4))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 *  returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)       ((x == GPIOA)?0:\
										                    (x == GPIOB)?1:\
										                    (x == GPIOC)?2:\
										                    (x == GPIOD)?3:\
								                        (x == GPIOE)?4:\
								                        (x == GPIOH)?7:0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		  6
#define IRQ_NO_EXTI1 		  7
#define IRQ_NO_EXTI2 		  8
#define IRQ_NO_EXTI3 		  9
#define IRQ_NO_EXTI4 		  10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15   15

/* Generic macros*/
#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET      ENABLE
#define GPIO_PIN_RESET    DISABLE
#define FLAG_SET          ENABLE
#define FLAG_RESET        DISABLE

/* Bit position definition of SPI Peripheral */
/* SPI control register 1 CR1 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15
/* SPI control register 2 CR2 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7
/* SPI status register SR */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

#endif /* STM32F411XX_H_ */
