#include <stm32f407xx_gpio_driver.h>
#include	"button_stm32f407xx_driver.h"

/*********************************************************************************************/
/*																				Private Variables																	 */
/*********************************************************************************************/

/* Button State */
button_state_t button_state = RELEASED_STATE;

/* Initialization structure */

gpio_pin_conf_t button_pin_conf = \
																	{\
																		BUTTON_PIN,\
																		GPIO_PIN_MODE_INPUT,\
																		GPIO_PIN_OP_TYPE_PUSHPULL,\
																		GPIO_PIN_NO_PULL,\
																		GPIO_PIN_SPEED_VERY_HIGH,\
																		GPIO_PIN_ALT_FUN_GPIO\
																	};

/*********************************************************************************************/
/*																				Exposed APIs																			 */
/*********************************************************************************************/
/* Button Initialization API */

void vButton_Init(void)
{
	_HAL_GPIOA_CLK_ENABLE();
	GPIO_Init(GPIOA, &button_pin_conf);
	GPIO_int_edg_sel(BUTTON_PIN, INT_RISING_FALLING_EDGE);

	button_state = RELEASED_STATE;
	
}
