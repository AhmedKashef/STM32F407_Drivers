#include <stm32f407xx_gpio_driver.h>
#include "stm32f4xx.h"
/****************************************************************************/
/* 							static helper Functions  						*/
/****************************************************************************/


/*function config. mode of the pin*/

#define GPIO_ConfPinMode(GPIOx, PIN_NO, MODE)	(GPIO_WriteRegMask(GPIOx->MODER, MODE, (PIN_NO * 2), 3))

/*function config OP type of the pin */

#define GPIO_ConfPinOType(GPIOx, PIN_NO, OP_TYPE)	(GPIO_WriteReg(GPIOx->OTYPER, OP_TYPE, PIN_NO))

/*function config. speed of the pin*/

#define GPIO_ConfPinSpeed(GPIOx, PIN_NO, SPEED)	(GPIO_WriteRegMask(GPIOx->OSPEEDR, SPEED, (PIN_NO * 2), 3))

/*function config. the pull up/pull dwn of the pin */

#define GPIO_ConfPinPUPD(GPIOx, PIN_NO, PUPD)	(GPIO_WriteRegMask(GPIOx->PUPDR, PUPD, (PIN_NO * 2), 3))

/*function config. the alternate function of the pin */

#define GPIO_ConfPinAF(GPIOx, PIN_NO, AF)	((7 > PIN_NO)?(GPIO_WriteRegMask(GPIOx->AFR[0], AF, (PIN_NO * 4), 0xF)):(GPIO_WriteRegMask(GPIOx->AFR[1], AF, ((PIN_NO % 8) * 4), 0xF)))
	

/****************************************************************************/
/* 										 Driver exposed APIs			 	  												*/
/****************************************************************************/

/* GPIO initilization api */
void GPIO_Init(GPIO_TypeDef* GPIOx, gpio_pin_conf_t* gpio_pin_conf)
{
	GPIO_ConfPinMode(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->mode);
	GPIO_ConfPinSpeed(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->speed);
	GPIO_ConfPinOType(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->op_type);
	GPIO_ConfPinPUPD(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->pull);
	GPIO_ConfPinAF(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->alternate);

}
/*GPIO Read from pin API*/
uint8_t GPIO_read_from_pin(GPIO_TypeDef* GPIOx, uint16_t pin_no)
{
	uint8_t ret;
	ret = ((GPIOx->IDR >> pin_no) & 0x00000001U);
	return ret;
	
}
/*GPIO Write to pin API*/
void GPIO_write_to_pin(GPIO_TypeDef* GPIOx, uint16_t pin_no, uint8_t value)
{
	if(0x01U & value)
	{
		GPIOx->ODR &= ~(STD_HIGH << pin_no);
		GPIOx->ODR |= (value << pin_no);
	}
		else
	{
		GPIOx->ODR &= ~(STD_HIGH << pin_no);
		GPIOx->ODR |= (value << pin_no);
	}

}

/* GPIO CONFIG. edge select Interrupt API*/

void GPIO_int_edg_sel(uint16_t pin_no, int_edge_sel_t edge_sel)
{
	switch(edge_sel)
	{
		case INT_RISING_EDGE:
			EXTI->RTSR |= (1U << pin_no);
		break;
		case INT_FALLING_EDGE:
			EXTI->FTSR |= (1U << pin_no);
		case INT_RISING_FALLING_EDGE:
			EXTI->RTSR |= (1U << pin_no);
			EXTI->FTSR |= (1U << pin_no);
		break;
		default:
			/* do nothing*/
		break;
	}
	
}

/*GPIO Interrupt Enable*/

void GPIO_int_enable(uint16_t pin_no, IRQn_Type irq )
{
	EXTI->IMR |= (1U << pin_no);	// enable interrupt register of EXTI controller
	_HAL_GPIO_INT_ENABLE(irq);
	
}

/*GPIO Interrupt Clear Pending API*/

void GPIO_int_clr_pd(uint16_t pin_no)
{
	if((EXTI->PR) & (1 << pin_no))
	{
		EXTI->PR |= (1 << pin_no); // clear pending interrupt by writing 1 to PR 
	}
	else{/*DO NOTHING*/}
	
}
