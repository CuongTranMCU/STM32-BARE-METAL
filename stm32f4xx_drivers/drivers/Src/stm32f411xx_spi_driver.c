/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Aug 27, 2025
 *      Author: dell
 */

#include "stm32f411xx_spi_driver.h"

/*****************************************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPIx
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (ENABLE == EnorDi)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}

/*****************************************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initializes the given SPIx peripheral
 *
 * @param[in]         - pointer to SPI handle structure
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    /* Enable the SPI peripheral clock */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    /* 1. Configure device mode */
    pSPIHandle->pSPIx->CR1 |=   pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    /* 2. Configure bus config */
    if (SPI_BUS_CONFIG_FD == pSPIHandle->SPIConfig.SPI_BusConfig)
    {
        /* BIDI mode should be cleared */
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (SPI_BUS_CONFIG_HD == pSPIHandle->SPIConfig.SPI_BusConfig)
    {
        /* BIDI mode should be set */
        pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (SPI_BUS_CONFIG_SIMPLEX_RXONLY == pSPIHandle->SPIConfig.SPI_BusConfig)
    {
        /* BIDI mode should be cleared */
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
        /* RXONLY bit must be set */
        pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);
    }

    /* 3. Configure SCLK speed (baud rate) */
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    /* 4. Configure DFF */
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    /* 5. Configure CPOL */
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    /* 6. Configure CPHA */
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    /* 7. Configure SSM */
    pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    /* 8. Enable the SPI peripheral */
    pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_SPE);
}

/*****************************************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function de-initializes the given SPIx peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
    else if (pSPIx == SPI4)
    {
        SPI4_REG_RESET();
    }
}

/*****************************************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function returns the status of the given flag
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - flag name
 * 
 * @return            - FLAG_SET or FLAG_RESET macros
 *
 * @Note              - none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if (pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*****************************************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data through SPIx peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to Tx buffer
 * @param[in]         - length of data
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        /* Wait until TXE = 1 */
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        /* Check the DFF bit in CR1 */
        if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) )
        {
            /* 16 bit DFF */
            /* Load the data into the DR register */
            pSPIx->DR = *((uint16_t*)pTxBuffer); // Cast to uint16_t pointer and dereference
            Len -= 2;
            (uint16_t*)pTxBuffer++; // Increment by 2 bytes
        }
        else
        {
            /* 8 bit DFF */
            /* Load the data into the DR register */
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
    /* Wait until SPI is not busy */
    while(SPI_GetFlagStatus(pSPIx, SPI_SR_BSY) == FLAG_SET);
}

/*****************************************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function receives data through SPIx peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to Rx buffer
 * @param[in]         - length of data
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

/*****************************************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - This function sends data through SPIx peripheral using interrupt
 *
 * @param[in]         - pointer to SPI handle structure
 * @param[in]         - pointer to Tx buffer
 * @param[in]         - length of data
 * 
 * @return            - state of SPI peripheral (busy or ready)
 *
 * @Note              - none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
    return 0;
}

/*****************************************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - This function receives data through SPIx peripheral using interrupt
 *
 * @param[in]         - pointer to SPI handle structure
 * @param[in]         - pointer to Rx buffer
 * @param[in]         - length of data
 * 
 * @return            - state of SPI peripheral (busy or ready)
 *
 * @Note              - none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    return 0;
}

/*****************************************************************************************
 * @fn      		  - SPI_IRQITConfig
 *
 * @brief             - This function configures the IRQ number and IRQ priority
 *
 * @param[in]         - IRQ number
 * @param[in]         - IRQ priority
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/*****************************************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function configures the IRQ priority
 *
 * @param[in]         - IRQ number
 * @param[in]         - IRQ priority
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/*****************************************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function handles the IRQ
 *
 * @param[in]         - pointer to SPI handle structure
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

/*****************************************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - This function configures the SSI bit in CR1 register
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * 
 * @return            - none
 *
 * @Note              - none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (ENABLE == EnorDi)
    {
        /* Set the SSI bit */
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        /* Clear the SSI bit */
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

