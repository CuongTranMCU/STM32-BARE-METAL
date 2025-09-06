
#include <stdint.h>

#define RCC_BASE_ADDR					0x40023800UL			// Enable SYSCFG by APB2
#define RCC_SYSCFG_OFFSET				0x44UL
#define RCC_SYSCFG_ADDR				    (RCC_BASE_ADDR + RCC_SYSCFG_OFFSET)
#define RCC_AHB1ENR_OFFSET				0x30UL			// Enable GPIOA by AHB1
#define RCC_AHB1ENR_REG_ADDR			(RCC_BASE_ADDR + RCC_AHB1ENR_OFFSET)
#define GPIOA_BASE_ADDR					0x40020000UL			// GPIOC base address
#define GPIOA_MODER_OFFSET				0x00UL			// GPIOA mode register
#define GPIOC_MODER_REG_ADDR			(GPIOC_BASE_ADDR + GPIOA_MODER_OFFSET)
#define NVIC_ISER						(0xE000E100UL)
#define SYSCFG_BASE_ADDR				(0x40013800UL)
#define SYSCFG_EXICR1_ADDR				(SYSCFG_BASE_ADDR + 0x08UL)
#define EXTI_BASE_ADDR 					(0x40013C00UL)
uint8_t volatile g_button_pressed = 0;
uint32_t g_button_press_count = 0;
uint32_t *pRccSyscfg = (uint32_t *)RCC_SYSCFG_ADDR;
uint32_t *pRccAhb1Enr = (uint32_t *)GPIOA_BASE_ADDR;
uint32_t *pSyscfgExtiCr1 = (uint32_t *)SYSCFG_EXICR1_ADDR;
uint32_t *pExtiIntMask = (uint32_t *)EXTI_BASE_ADDR;
uint32_t *pExtiRising = (uint32_t *)(EXTI_BASE_ADDR + 0x08);
uint32_t *pExtiPending = (uint32_t *)(EXTI_BASE_ADDR + 0x14);
uint32_t *pNVICIntEn = (uint32_t *)(NVIC_ISER);
void EXTI0_IRQHandler()
{
	g_button_pressed = 1;
	// write 1 to clear:
	*pExtiPending |= (1 << 0);

}
int main()
{
	// Enable clock for SYSCFG
	*pRccSyscfg |= (1 << 14);
	// Enable Clock for GPIOA
	*pRccAhb1Enr |= (1 << 0);
	// Set PA0 is input mode
	*pRccAhb1Enr &= ~(1 << 0);
	*pRccAhb1Enr &= ~(1 << 1);
	// set EXTI0 is PA0
	*pSyscfgExtiCr1 &= (0xF << 0);
	// unmask line 0
	*pExtiIntMask |= (1 << 0);
	// rising edge
	*pExtiRising |= (1 << 0);
	// enable IRQ6
	*pNVICIntEn |= (1 << 6);
	while(1)
	{
		if (g_button_pressed == 1)
		{
			g_button_press_count++;
		}
		g_button_pressed = 0;
	}

    return 0;
}


