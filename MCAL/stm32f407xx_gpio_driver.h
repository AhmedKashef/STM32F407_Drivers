#ifndef __STM32F407XX_GPIO_DRIVER__
#define	__STM32F407XX_GPIO_DRIVER__
#include "stm32f4xx.h"
/***************************************************************************/
/* 									1- Macros Used for GPIO Initlization 							 		*/
/***************************************************************************/
/*GPIO Mode Setting Values*/

#define GPIO_PIN_MODE_INPUT		((uint32_t)0x0U)
#define GPIO_PIN_MODE_OUTPUT		((uint32_t)0x1U)
#define GPIO_PIN_MODE_ALT_FUN		((uint32_t)0x2U)
#define GPIO_PIN_MODE_ANALOG		((uint32_t)0x3U)

/*GPIO OP type selection values*/

#define GPIO_PIN_OP_TYPE_PUSHPULL		((uint32_t)0x0U)
#define GPIO_PIN_OP_TYPE_OPEN_DRAIN ((uint32_t)0x1U)

/*GPIO Speed type selection values */

#define GPIO_PIN_SPEED_LOW					((uint32_t)0x0U)
#define GPIO_PIN_SPEED_MEDIUM				((uint32_t)0x1U)
#define GPIO_PIN_SPEED_HIGH					((uint32_t)0x2U)
#define GPIO_PIN_SPEED_VERY_HIGH		((uint32_t)0x3U)

/*GPIO pull up/pull dwn selection values */
#define GPIO_PIN_NO_PULL    ((uint32_t)0x0U)
#define GPIO_PIN_PULL_UP    ((uint32_t)0x1U)
#define GPIO_PIN_PULL_DOWN	((uint32_t)0x2U)

/*GPIO ALTERNATE FUNCTTION selection values*/

#define	GPIO_PIN_ALT_FUN_GPIO										((uint32_t)0U)
#define	GPIO_PIN_ALT_FUN_TIM_1_2                ((uint32_t)1U)
#define	GPIO_PIN_ALT_FUN_TIM_3_5                ((uint32_t)2U)
#define	GPIO_PIN_ALT_FUN_TIM_8_11               ((uint32_t)3U)
#define	GPIO_PIN_ALT_FUN_I2C_1_3                ((uint32_t)4U)
#define	GPIO_PIN_ALT_FUN_SPI_1_2                ((uint32_t)5U)
#define	GPIO_PIN_ALT_FUN_SPI_3                  ((uint32_t)6U)
#define	GPIO_PIN_ALT_FUN_USART_1_3              ((uint32_t)7U)
#define	GPIO_PIN_ALT_FUN_USART_4_6              ((uint32_t)8U)
#define	GPIO_PIN_ALT_FUN_CAN_1_2_TIM_12_14      ((uint32_t)9U)
#define	GPIO_PIN_ALT_FUN_OTG_FS_OTG_HS          ((uint32_t)10U)
#define	GPIO_PIN_ALT_FUN_ETH                    ((uint32_t)11U)
#define	GPIO_PIN_ALT_FUN_FSMC_SDIO_OTG_HS       ((uint32_t)12U)
#define	GPIO_PIN_ALT_FUN_DCMI                   ((uint32_t)13U)
#define	GPIO_PIN_ALT_FUN__                      ((uint32_t)14U)
#define	GPIO_PIN_ALT_FUN_EVENTOUT               ((uint32_t)15U)

/*gpio port address */

#define GPIO_PORT_A		GPIOA
#define GPIO_PORT_B		GPIOB
#define GPIO_PORT_C		GPIOC
#define GPIO_PORT_D   GPIOD
#define GPIO_PORT_E   GPIOE
#define GPIO_PORT_F   GPIOF
#define GPIO_PORT_G   GPIOG
#define GPIO_PORT_H   GPIOH
#define GPIO_PORT_I   GPIOI

/*gpio port pins */

#define GPIO_PIN_0		(0U)
#define GPIO_PIN_1		(1U)
#define GPIO_PIN_2		(2U)
#define GPIO_PIN_3  	(3U)
#define GPIO_PIN_4  	(4U)
#define GPIO_PIN_5  	(5U)
#define GPIO_PIN_6  	(6U)
#define GPIO_PIN_7  	(7U)
#define GPIO_PIN_8  	(8U)
#define GPIO_PIN_9  	(9U)
#define GPIO_PIN_10  	(10U)
#define GPIO_PIN_11  	(11U)
#define GPIO_PIN_12 	(12U)
#define GPIO_PIN_13 	(13U)
#define GPIO_PIN_14 	(14U)
#define GPIO_PIN_15 	(15U)

/* STD Values */

#define STD_LOW		(0U)
#define STD_HIGH  (1U)

/* GPIO IRQ */

#define GPIO_IRQ_PIN_0	(EXTI0_IRQn)
#define GPIO_IRQ_PIN_1	(EXTI1_IRQn)
#define GPIO_IRQ_PIN_2	(EXTI2_IRQn)
#define GPIO_IRQ_PIN_3	(EXTI3_IRQn)
#define GPIO_IRQ_PIN_4	(EXTI4_IRQn)
#define GPIO_IRQ_PIN_5	(EXTI9_5_IRQn)
#define GPIO_IRQ_PIN_6	(EXTI9_5_IRQn)
#define GPIO_IRQ_PIN_7	(EXTI9_5_IRQn)
#define GPIO_IRQ_PIN_8	(EXTI9_5_IRQn)
#define GPIO_IRQ_PIN_9	(EXTI9_5_IRQn)
#define GPIO_IRQ_PIN_10	(EXTI15_10_IRQn)
#define GPIO_IRQ_PIN_11	(EXTI15_10_IRQn)
#define GPIO_IRQ_PIN_12	(EXTI15_10_IRQn)
#define GPIO_IRQ_PIN_13	(EXTI15_10_IRQn)
#define GPIO_IRQ_PIN_14	(EXTI15_10_IRQn)
#define GPIO_IRQ_PIN_15	(EXTI15_10_IRQn)


/* GPIO Write Read MACROS */
#define GPIO_WriteReg(REG, DATA, BIT)					( (REG) = ( (((REG) & ~(1U<<(BIT)))|((DATA))<<(BIT))) )
#define GPIO_WriteRegMask(REG, DATA, BIT, MASK)			( (REG) = ( (((REG) & ~((MASK)<<(BIT)))|((DATA))<<(BIT))) )
#define GPIO_ReadReg(REG, BIT)							( ((REG)&((1U)<<(BIT)))>>(BIT) )
/*Macro GPIO Enable Interrupt */

#define _HAL_GPIO_INT_ENABLE(GPIO_IRQ_PIN)	(NVIC_EnableIRQ((GPIO_IRQ_PIN)))

/*Macro GPIO Disable Interrupt */

#define _HAL_GPIO_INT_DISABLE(GPIO_IRQ_PIN)	(__NVIC_DisableIRQ(GPIO_IRQ_PIN))

/* macros to enable clock for different GPIO ports in RCC register */

#define _HAL_GPIOA_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 0))
#define _HAL_GPIOB_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 1))
#define _HAL_GPIOC_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 2))
#define _HAL_GPIOD_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 3))
#define _HAL_GPIOE_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 4))
#define _HAL_GPIOF_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 5))
#define _HAL_GPIOG_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 6))
#define _HAL_GPIOH_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 7))
#define _HAL_GPIOI_CLK_ENABLE()		((RCC->AHB1ENR) |= (1U << 8))

/****************************************************************************/
/* 									2- Data Structures for GPIO pin Initilization			 	  	*/
/****************************************************************************/

/* Configuration Structures */
typedef struct
{
	uint32_t pin; 				/*specify gpio pin to be configured */
	uint32_t mode;				/*specify mode of the selected pin*/ 
	uint32_t op_type; 		/*specify op type of the selected pin*/ 
	uint32_t pull;				/*specify Pull up/pull dwn of the selected pin*/ 
	uint32_t speed;				/*specify speed of the selected pin*/
	uint32_t alternate;		/*specify alterante function of the selected pin if the mode selected is alternate*/
}gpio_pin_conf_t;

/* GPIO Interrupt Edge Selection strucuture*/

typedef enum
{
	INT_RISING_EDGE,
	INT_FALLING_EDGE,
	INT_RISING_FALLING_EDGE
}int_edge_sel_t;

/****************************************************************************/
/* 									3- Driver exposed APIs			 	  												*/
/****************************************************************************/
/* GPIO initilization api */

void GPIO_Init(GPIO_TypeDef* GPIOx, gpio_pin_conf_t* gpio_pin_conf);

/*GPIO Read from pin API*/

uint8_t GPIO_read_from_pin(GPIO_TypeDef* GPIOx, uint16_t pin_no);

/*GPIO Write to pin*/

void GPIO_write_to_pin(GPIO_TypeDef* GPIOx, uint16_t pin_no, uint8_t value);

/*GPIO Interrupt Enable*/

void GPIO_int_enable(uint16_t pin_no, IRQn_Type irq );

/* GPIO CONFIG. edge select Interrupt API*/

void GPIO_int_edg_sel(uint16_t pin_no, int_edge_sel_t edge_sel);
	
/*GPIO Interrupt Clear Pending API*/

void GPIO_int_clr_pd(uint16_t pin_no);
	
#endif //__STM32F407XX_GPIO_DRIVER__
