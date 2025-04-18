/* All libraries ---------------------------------------------------*/
#include "stm32f401re_gpio.h"
#include "stm32f401re_rcc.h"
#include <stdint.h>

/* Private macro ---------------------------------------------------*/
#define BUZZER_GPIO_PORT				GPIOC
#define BUZZER_GPIO_PIN					GPIO_Pin_9
#define BUZZER_PIN						9
#define BUZZERControl_SetClock			RCC_AHB1Periph_GPIOC

#define BUTTON_GPIO_PORT				GPIOB
#define BUTTON_GPIO_PIN					GPIO_Pin_4
#define BUTTON_PIN						4
#define BUTTONControl_SetClock			RCC_AHB1Periph_GPIOB

#define GPIO_PIN_SET				1
#define GPIO_PIN_RESET				0

static void Buzzer_Init(void);
static void Button_Init(void);
static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint32_t GPIO_PIN);
static void BuzzerControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t Status);

int main()
{
	Buzzer_Init();
	Button_Init();

	while(1)
	{
		if(ButtonRead_Status(BUTTON_GPIO_PORT, BUTTON_PIN) == 0)
		{
			BuzzerControl_SetStatus(BUZZER_GPIO_PORT, BUZZER_PIN, 1);
		}
		else
		{
			BuzzerControl_SetStatus(BUZZER_GPIO_PORT, BUZZER_PIN, 0);
		}
	}

	return 0;
}

static void Buzzer_Init(void)
{
	//To create a variable of GPIO Handle
	GPIO_InitTypeDef 	GPIO_Buzzer_InitStructure;

	//To enable clock pulse at GPIOA port
	RCC_AHB1PeriphClockCmd(BUZZERControl_SetClock, ENABLE);

	//To set up all values of all fields of the structure type GPIO_PinConfig_t
	GPIO_Buzzer_InitStructure.GPIO_Pin = BUZZER_GPIO_PIN;			//To select pin to configure
	GPIO_Buzzer_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//To configure the mode "Output" of that pin
	GPIO_Buzzer_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//To configure the process speed at that pin
	GPIO_Buzzer_InitStructure.GPIO_OType = GPIO_OType_PP;			//To configure the output mode "Open-Drain" at that pin
	GPIO_Buzzer_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//Do not use PU-PD
	//To initialize all values
	GPIO_Init(BUZZER_GPIO_PORT, &GPIO_Buzzer_InitStructure);
}

static void Button_Init(void)
{
	//To create a variable of GPIO Handle
	GPIO_InitTypeDef	GPIO_Button_InitStructure;

	//To enable clock pulse at GPIOA port
	RCC_AHB1PeriphClockCmd(BUTTONControl_SetClock, ENABLE);

	//To set up all values of all fields of the structure type GPIO_PinConfig_t
	GPIO_Button_InitStructure.GPIO_Pin = BUTTON_GPIO_PIN;		//To select pin to configure
	GPIO_Button_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//To configure the mode "Output" of that pin
	GPIO_Button_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//Do not use PU-PD
	//To initialize all values
	GPIO_Init(BUTTON_GPIO_PORT, &GPIO_Button_InitStructure);
}

static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint32_t GPIO_PIN)
{
	uint32_t Read_Pin;
	Read_Pin = (GPIOx->IDR) >> GPIO_PIN;
	Read_Pin = Read_Pin & 0x01;
	return Read_Pin;
}

static void BuzzerControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t Status)
{
	//Set bit in BSRR Registers
	if(Status == GPIO_PIN_SET)
	{
		GPIOx->BSRRL |= 1 << GPIO_PIN;
	}
	//Reset bit in BSRR Registers
	if(Status == GPIO_PIN_RESET)
	{
		GPIOx->BSRRH |= 1 << GPIO_PIN;
	}
}
