
#include <stdint.h>

#define I2C2_BASE_ADDR					0x40005800UL
#define I2C_OAR2_OFFSET					0x0CUL
#define I2C2_OAR2_REG_ADDR				(I2C2_BASE_ADDR + I2C_OAR2_OFFSET)
#define RCC_BASE_ADDR					0x40023800UL			// Enable I2C2 by APB1
#define RCC_APB1ENR_OFFSET				0x40UL
#define RCC_APB1ENR_REG_ADDR			(RCC_BASE_ADDR + RCC_APB1ENR_OFFSET)

int main()
{
	uint32_t *pI2cOar2Reg =		(uint32_t*) I2C2_OAR2_REG_ADDR;
	uint32_t *pRccApb1EnrReg =  (uint32_t*) RCC_APB1ENR_REG_ADDR;
	/* Enable I2C2 by APB1ENR*/
	*pRccApb1EnrReg |= (1 << 22);
	/* Set ENDUAL = 1 */
	*pI2cOar2Reg |= 1;

	while(1);
    return 0;
}

