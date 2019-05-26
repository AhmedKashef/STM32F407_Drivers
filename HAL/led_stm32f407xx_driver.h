#ifndef	__LED_STM32F407XX_H__
#define	__LED_STM32F407XX_H__

#include <stm32f407xx_gpio_driver.h>

/******************************************************************************************/
/*																		1-MACROS & CONST.																		*/
/******************************************************************************************/

/* LED ports */

#define GPIO_LED_GREEN_PORT			(GPIO_PORT_D)
#define GPIO_LED_ORANGE_PORT		(GPIO_PORT_D)
#define GPIO_LED_RED_PORT				(GPIO_PORT_D)
#define GPIO_LED_BLUE_PORT			(GPIO_PORT_D)

/* LED PINS */
	
#define GPIO_LED_GREEN_PIN			(GPIO_PIN_12)
#define GPIO_LED_ORANGE_PIN     (GPIO_PIN_13)
#define GPIO_LED_RED_PIN		    (GPIO_PIN_14)
#define GPIO_LED_BLUE_PIN	      (GPIO_PIN_15)

/* LED STATES */

#define LED_OFF	(STD_LOW)
#define LED_ON	(STD_HIGH)

/* Enable clock to LEDS */

#define _HAL_LEDS_CLK_ENABLE()							_HAL_GPIOD_CLK_ENABLE()

/******************************************************************************************/
/*																		2-Data Structures																		*/
/******************************************************************************************/

/* Led States structure*/

typedef enum
{
	led_state_off = 0U,
	led_state_on 	= 1U
}led_state_t;


/******************************************************************************************/
/*																		3-LED Exposed APIs																	*/
/******************************************************************************************/

/* LED Init. API */

void vLed_Init(void);

/* LED turn on API */

void vLed_TurnOn(GPIO_TypeDef *GPIOx, uint16_t pin);

/*LED turn off API */

void vLed_TurnOff (GPIO_TypeDef *GPIOx, uint16_t pin);

/*LED toggle */

void vLed_Toggle(GPIO_TypeDef *GPIOx, uint16_t pin);


#endif	//__LED_STM32F407XX_H__
