/* All libraries ---------------------------------------------------*/
#include <stdio.h>

/* Private macro ---------------------------------------------------*/
#define RCC_BASE_ADDR         	0x40023800UL //Basic address of RCC peripheral
#define RCC_CFGR_REG_OFFSET   	0x08UL //Offset address of RCC_CFGR
#define RCC_CFGR_REG_ADDR     	(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET) //Address of RCC_CFGR

#define GPIOA_BASE_ADDR       	0x40020000UL //Basic address of GPIOA

int main()
{
	//Declaring a pointer to point the address of RCC_CFGR
	uint32_t *pRccCfgrReg = (uint32_t *)RCC_CFGR_REG_ADDR;

	//To change the system clock to HSI
	*pRccCfgrReg &= ~(0x3 << 21);

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
