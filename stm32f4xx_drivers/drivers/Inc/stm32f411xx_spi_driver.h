/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Aug 27, 2025
 *      Author: dell
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

/*
 * @brief structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;         /*< Specifies the SPI operating mode: master or slave >*/
	uint8_t SPI_BusConfig;          /*< Specifies the SPI bus configuration: full-duplex, half-duplex or simplex >*/    
	uint8_t SPI_SclkSpeed;          /*< Specifies the SPI serial clock speed >*/
	uint8_t SPI_DFF;                /*< Specifies the SPI data frame format: 8-bit or 16-bit >*/                                                           
	uint8_t SPI_CPOL;               /*< Specifies the SPI clock polarity >*/
	uint8_t SPI_CPHA;               /*< Specifies the SPI clock phase >*/
	uint8_t SPI_SSM;                /*< Specifies the SPI software slave management >*/                         
}SPI_Config_t;

/* 
 * @brief Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;     /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
	SPI_Config_t 	SPIConfig;  /*< SPI configuration settings >*/
	// uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	// uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	// uint32_t 		TxLen;		/* !< To store Tx len > */
	// uint32_t 		RxLen;		/* !< To store Tx len > */
	// uint8_t 		TxState;	/* !< To store Tx state > */
	// uint8_t 		RxState;	/* !< To store Rx state > */
}SPI_Handle_t;

/**
 * @brief SPI Device Mode
 */
typedef enum
{
    SPI_DEVICE_MODE_SLAVE = 0,
    SPI_DEVICE_MODE_MASTER
} SPI_DeviceMode_t;

/**
 * @brief SPI Bus Configuration
 */
typedef enum
{
    SPI_BUS_CONFIG_FD = 0,
    SPI_BUS_CONFIG_HD,
    SPI_BUS_CONFIG_SIMPLEX_TXONLY,  /* Not used in this driver */
    SPI_BUS_CONFIG_SIMPLEX_RXONLY
} SPI_BusConfig_t;

/**
 * @brief SPI Serial Clock Speed
 */
typedef enum
{
    SPI_SCLK_SPEED_DIV2 = 0,
    SPI_SCLK_SPEED_DIV4,
    SPI_SCLK_SPEED_DIV8,
    SPI_SCLK_SPEED_DIV16,
    SPI_SCLK_SPEED_DIV32,
    SPI_SCLK_SPEED_DIV64,
    SPI_SCLK_SPEED_DIV128,
    SPI_SCLK_SPEED_DIV256
} SPI_SclkSpeed_t;

/**
 * @brief SPI Data Frame Format
 */
typedef enum
{
    SPI_DFF_8BITS = 0,
    SPI_DFF_16BITS
} SPI_DFF_t;

/**
 * @brief SPI Clock Polarity
 */
typedef enum
{
    SPI_CPOL_LOW = 0,
    SPI_CPOL_HIGH
} SPI_CPOL_t;

/**
 * @brief SPI Clock Phase
 */
typedef enum
{
    SPI_CPHA_LOW = 0,
    SPI_CPHA_HIGH
} SPI_CPHA_t;

/**
 * @brief SPI Software Slave Management
 */
typedef enum
{
    SPI_SSM_DI = 0,
    SPI_SSM_EN
} SPI_SSM_t;

/* SPI related status flags definitions */
#define SPI_TXE_FLAG        (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG       (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG       (1 << SPI_SR_BSY)
/***************************************************************************************************************************************************************
 *                                                          API supported by this driver
 *                                          For more information about the APIs check the function definitions
 * 
 **************************************************************************************************************************************************************/

/**
 * @brief Peripheral Clock setup
 * 
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief Init and De-init
 * 
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief Data Send and Receive
 * 
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * @brief IRQ Configuration and ISR handling
 * 
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/**
 * @brief Other Peripheral Control APIs
 * 
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
