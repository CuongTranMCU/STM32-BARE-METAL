
#include <stdint.h>

#define RCC_BASE_ADDR					0x40023800UL			// Enable I2C2 by APB1
#define RCC_CFGR_OFFSET					0x08UL
#define RCC_CR_OFFSET		  			0x00UL
#define RCC_CR_REG_ADDR				    (RCC_BASE_ADDR + RCC_CR_OFFSET)
#define RCC_CFGR_REG_ADDR				(RCC_BASE_ADDR + RCC_CFGR_OFFSET)
#define RCC_AHB1ENR_OFFSET				0x30UL			// Enable GPIOC by AHB1
#define RCC_AHB1ENR_REG_ADDR			(RCC_BASE_ADDR + RCC_AHB1ENR_OFFSET)
#define GPIOC_BASE_ADDR					0x40020800UL			// GPIOC base address
#define GPIOC_MODER_OFFSET				0x00UL			// GPIOC mode register
#define GPIOC_MODER_REG_ADDR			(GPIOC_BASE_ADDR + GPIOC_MODER_OFFSET)
#define GPIOC_AFRH_OFFSET				0x24UL			// GPIOC alternate function high register: PC9(8->15-> high) (MCO_2)
#define GPIOC_AFRH_REG_ADDR				(GPIOC_BASE_ADDR + GPIOC_AFRH_OFFSET)

int main()
{
	uint32_t *pRccCrReg = (uint32_t*)RCC_CR_REG_ADDR;
	*pRccCrReg |= (1 << 16); // Enable HSE clock using HSE ON bit
	// Wait until HSE clock from the external crystal stabilizes (only if crystal is connected)
	while(!(*pRccCrReg & (1 << 17)));
	// Switch the system clock to HSE (RCC_CFGR)
	uint32_t *pRccCfgrReg =  (uint32_t*) RCC_CFGR_REG_ADDR;
	/* Config MCO_2 to HSE: 10 */
	*pRccCfgrReg |= (1 << 31); // Set bit 31 to select HSE as clock source for MCO_2
	/* Configure MCO_2 prescaler to divide by 4: 110 */
	*pRccCfgrReg |= (1 << 28); // Set bit 28
	*pRccCfgrReg |= (1 << 29); // Set bit 29

	/* Config PC9 for MCO_2*/
	uint32_t *pRccAhb1Enr = (uint32_t*)RCC_AHB1ENR_REG_ADDR;
	/* Enable GPIOC for alternative function */
	*pRccAhb1Enr |= (1 << 2); // Enable GPIOC peripheral clock
	uint32_t *pGPIOCModeReg = (uint32_t*)(GPIOC_MODER_REG_ADDR);
	*pGPIOCModeReg &= ~( 0x3 << 18); //clear
	*pGPIOCModeReg |= ( 0x2 << 18);  //set
	
	/* Configure Alternative High Register*/
	uint32_t* pGPIOCAfrhReg = (uint32_t* ) GPIOC_AFRH_REG_ADDR;
	*pGPIOCAfrhReg &= ~(0xF << 4); // Clear bits 4 to 7 for PC9
	while(1);
    return 0;
}

