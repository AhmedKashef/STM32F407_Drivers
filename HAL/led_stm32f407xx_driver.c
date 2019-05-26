#include <stm32f407xx_gpio_driver.h>
#include "led_stm32f407xx_driver.h"

/******************************************************************************************/
/*																	local Datatypes																				*/
/******************************************************************************************/

/* Initalize state */

static led_state_t led_green_state = led_state_off;
static led_state_t led_orange_state = led_state_off;
static led_state_t led_red_state = led_state_off;
static led_state_t led_blue_state = led_state_off;

/* gpio config of the led pins */

static gpio_pin_conf_t gpio_led_green_conf = \
																						 { 
																							 GPIO_LED_GREEN_PIN,\
																							 GPIO_PIN_MODE_OUTPUT,\
																							 GPIO_PIN_OP_TYPE_PUSHPULL,\
																							 GPIO_PIN_NO_PULL,\
																							 GPIO_PIN_SPEED_LOW,\
																							 GPIO_PIN_ALT_FUN_GPIO\
																						 };

static gpio_pin_conf_t gpio_led_orange_conf = \
																						 { 
																							 GPIO_LED_ORANGE_PIN,\
																							 GPIO_PIN_MODE_OUTPUT,\
																							 GPIO_PIN_OP_TYPE_PUSHPULL,\
																							 GPIO_PIN_NO_PULL,\
																							 GPIO_PIN_SPEED_MEDIUM,\
																							 GPIO_PIN_ALT_FUN_GPIO\
																						 };																						 

static gpio_pin_conf_t gpio_led_red_conf = \
																						 { 
																							 GPIO_LED_RED_PIN,\
																							 GPIO_PIN_MODE_OUTPUT,\
																							 GPIO_PIN_OP_TYPE_PUSHPULL,\
																							 GPIO_PIN_NO_PULL,\
																							 GPIO_PIN_SPEED_HIGH,\
																							 GPIO_PIN_ALT_FUN_GPIO\
																						 };

static gpio_pin_conf_t gpio_led_blue_conf = \
																						 { 
																							 GPIO_LED_BLUE_PIN,\
																							 GPIO_PIN_MODE_OUTPUT,\
																							 GPIO_PIN_OP_TYPE_PUSHPULL,\
																							 GPIO_PIN_NO_PULL,\
																							 GPIO_PIN_SPEED_VERY_HIGH,\
																							 GPIO_PIN_ALT_FUN_GPIO\
																						 };		

/******************************************************************************************/
/*																	LED Exposed APIs																			*/
/******************************************************************************************/
																						 
/* LED Init. API */

void vLed_Init(void)
{
	_HAL_LEDS_CLK_ENABLE();
	
	GPIO_Init(GPIO_LED_GREEN_PORT, &gpio_led_green_conf);
	GPIO_Init(GPIO_LED_ORANGE_PORT, &gpio_led_orange_conf);
	GPIO_Init(GPIO_LED_RED_PORT, &gpio_led_red_conf);
	GPIO_Init(GPIO_LED_BLUE_PORT, &gpio_led_blue_conf);
	
	GPIO_write_to_pin(GPIO_LED_GREEN_PORT, GPIO_LED_GREEN_PIN, led_state_off);
	GPIO_write_to_pin(GPIO_LED_ORANGE_PORT, GPIO_LED_ORANGE_PIN, led_state_off);
	GPIO_write_to_pin(GPIO_LED_RED_PORT, GPIO_LED_RED_PIN, led_state_off);
	GPIO_write_to_pin(GPIO_LED_BLUE_PORT, GPIO_LED_BLUE_PIN, led_state_off);
	
	led_green_state = led_state_off;
	led_orange_state = led_state_off;	
	led_red_state = led_state_off;	
	led_blue_state = led_state_off;	
	
}

/* LED turn on API */

void vLed_TurnOn(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	switch (pin)
	{
		case GPIO_LED_GREEN_PIN:
			GPIO_write_to_pin(GPIOx, GPIO_LED_GREEN_PIN, led_state_on);
			led_green_state = led_state_on;
		break;
		
		case GPIO_LED_ORANGE_PIN:
			GPIO_write_to_pin(GPIOx, GPIO_LED_ORANGE_PIN, led_state_on);
			led_orange_state = led_state_on;
		break;
	
		case GPIO_LED_RED_PIN:
			GPIO_write_to_pin(GPIOx, GPIO_LED_RED_PIN, led_state_on);
			led_red_state = led_state_on;
		break;
		
		case GPIO_LED_BLUE_PIN:
			GPIO_write_to_pin(GPIOx, GPIO_LED_BLUE_PIN, led_state_on);
			led_blue_state = led_state_on;
		break;
		
		default: /* do nothing */
			break;
	}

}

/*LED turn off API */

void vLed_TurnOff (GPIO_TypeDef *GPIOx, uint16_t pin)
{
	switch (pin)
	{
		case GPIO_LED_GREEN_PIN:
			GPIO_write_to_pin(GPIOx, GPIO_LED_GREEN_PIN, led_state_off);
			led_green_state = led_state_off;
		break;
		
		case GPIO_LED_ORANGE_PIN:
			GPIO_write_to_pin(GPIOx, GPIO_LED_ORANGE_PIN, led_state_off);
			led_orange_state = led_state_off;
		break;
	
		case GPIO_LED_RED_PIN:
			GPIO_write_to_pin(GPIOx, GPIO_LED_RED_PIN, led_state_off);
			led_red_state = led_state_off;
		break;
		
		case GPIO_LED_BLUE_PIN:
			GPIO_write_to_pin(GPIOx, GPIO_LED_BLUE_PIN, led_state_off);
			led_blue_state = led_state_off;
		break;
		
		default: /* do nothing */
			break;
	}
}

/*LED toggle */

void vLed_Toggle(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	switch (pin)
	{
		case GPIO_LED_GREEN_PIN:
			if(led_green_state)
			{
				GPIO_write_to_pin(GPIOx, GPIO_LED_GREEN_PIN, LED_OFF);
				led_green_state = led_state_off;
			}
			else
			{
				GPIO_write_to_pin(GPIOx, GPIO_LED_GREEN_PIN, LED_ON);
				led_green_state = led_state_on;
			}
				break;
		
		case GPIO_LED_ORANGE_PIN:
			if(led_orange_state)
			{
				GPIO_write_to_pin(GPIOx, GPIO_LED_ORANGE_PIN, LED_OFF);
				led_orange_state = led_state_off;
			}
			else
			{
				GPIO_write_to_pin(GPIOx, GPIO_LED_ORANGE_PIN, LED_ON);
				led_orange_state = led_state_on;
			}
		break;
	
		case GPIO_LED_RED_PIN:
			if(led_red_state)
			{
				GPIO_write_to_pin(GPIOx, GPIO_LED_RED_PIN, LED_OFF);
				led_red_state = led_state_off;
			}
			else
			{
				GPIO_write_to_pin(GPIOx, GPIO_LED_RED_PIN, LED_ON);
				led_red_state = led_state_on;
			}
		break;
		
		case GPIO_LED_BLUE_PIN:
			if(led_blue_state)
			{
				GPIO_write_to_pin(GPIOx, GPIO_LED_BLUE_PIN, LED_OFF);
				led_blue_state = led_state_off;
			}
			else
			{
				GPIO_write_to_pin(GPIOx, GPIO_LED_BLUE_PIN, LED_ON);
				led_blue_state = led_state_on;
			}
		break;
		
		default: /* do nothing */
			break;
	}
}
