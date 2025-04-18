/* All libraries ---------------------------------------------------*/
#include <stdio.h>

/* Private macro ---------------------------------------------------*/
#define RCC_BASE_ADDR			0x40023800UL //Basic address of RCC peripheral
#define RCC_CFGR_REG_OFFSET		0x08UL //Offset address of RCC_CFGR
#define RCC_CFGR_REG_ADDR		(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET) //Address of RCC_CFGR

#define RCC_CR_REG_OFFSET		0x00UL //Offset address of RCC_CR
#define RCC_CR_REG_ADDR			(RCC_BASE_ADDR + RCC_CR_REG_OFFSET) //Address of RCC_CR

#define GPIOA_BASE_ADDR       	0x40020000UL //Basic address of GPIOA

int main()
{
	//Declaring a pointer to point the address of RCC_CFGR, RCC_CR
	uint32_t *pRccCfgrReg = (uint32_t *)RCC_CFGR_REG_ADDR;
	uint32_t *pRccCrReg = (uint32_t *)RCC_CR_REG_ADDR;

	//To allow HSE clock pulse to perform
	*pRccCrReg |= (1 << 16);

	//To wait until HSE clock pulse being
	while(!(*pRccCrReg & (1 << 17)));

	//To translate the system clock into HSE source
	*pRccCfgrReg |= (1 << 0);

	//To configure filed bit of MCO1 at HSE mode
	*pRccCfgrReg |= (1 << 22);

	//Setting factor of divisor is 4
	*pRccCfgrReg |= (1 << 25);
	*pRccCfgrReg |= (1 << 26);

	//To create clock pulse for GPIO A
	uint32_t *pRCCAhb1Enr = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCCAhb1Enr |= (1 << 0);

	//To set up the mode of PA8 pin at alternate mode
	uint32_t *pGPIOAModeReg = (uint32_t*)(GPIOA_BASE_ADDR + 00);
	*pGPIOAModeReg &= ~( 0x3 << 16);
	*pGPIOAModeReg |= ( 0x2 << 16);

	//To set up the register alternation function at mode 0
	uint32_t *pGPIOAAltFunHighReg = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);
	*pGPIOAAltFunHighReg &= ~( 0x0f << 0);
}
