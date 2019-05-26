#ifndef		_BUTTON_STM32F407xx_DRIVER_H_
#define		_BUTTON_STM32F407xx_DRIVER_H_

#include <stm32f407xx_gpio_driver.h>


/*********************************************************************************************/
/*																				1-MACROS & CONSTANTS															 */
/*********************************************************************************************/

/* Button PORT */

#define BUTTON_PORT		(GPIO_PORT_A)

/* Button PIN */

#define BUTTON_PIN		(GPIO_PIN_0)

/* BUTTON Interrupt IRQ */

#define BUTTON_IRQ	GPIO_IRQ_PIN_0

/*********************************************************************************************/
/*																				2-DATA Types & Structures													 */
/*********************************************************************************************/

/* Button States Structure */

typedef enum
{
	RELEASED_STATE,
	PRE_PRESSED_STATE,
	PRESSED_STATE,
	PRE_RELEASED_STATE
}button_state_t;

/*********************************************************************************************/
/*																				3-Driver EXPOSED APIs															 */
/*********************************************************************************************/

/* Button Initialization API */

void vButton_Init(void);


#endif //_BUTTON_STM32F407xx_DRIVER_H_
