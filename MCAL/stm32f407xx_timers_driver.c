/*
 * stm32f407xx_timers_driver.c
 *
 *  Created on: Feb 18, 2019
 *      Author: WE72 7RJ
 */
#include "stm32f407xx_timers_driver.h"

/******************* Private Macros ***********************************/

void BT_Init(BT_HandleStruct_t * btHandle)
{
	uint8_t i = 1;
	if(btHandle->state == BT_STATE_RESET)
	{
		/* mode Initialization */
		switch(btHandle->init->mode)
		{
		case BT_MODE_1SHOT_NO_EVENT:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_ENABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_DISABLE,BT_REG_CR1_UDIS);
			break;
		case BT_MODE_1SHOT_EVENT_OF:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_ENABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_ENABLE,BT_REG_CR1_UDIS);
			Timer_WriteBit(btHandle->instance->CR1,BT_URS_DISABLE,BT_REG_CR1_URS);
			break;
		case BT_MODE_1SHOT_EVENT_UG:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_ENABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_ENABLE,BT_REG_CR1_UDIS);
			Timer_WriteBit(btHandle->instance->CR1,BT_URS_DISABLE,BT_REG_CR1_URS);
			break;
		case BT_MODE_1SHOT_INTERRUPT_OF:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_ENABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_ENABLE,BT_REG_CR1_UDIS);
			Timer_WriteBit(btHandle->instance->CR1,BT_URS_DISABLE,BT_REG_CR1_URS);
			Timer_WriteBit(btHandle->instance->DIER,BT_UIE_ENABLE,BT_REG_DIER_UIE);
			break;
		case BT_MODE_1SHOT_INTERRUPT_UG:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_ENABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_ENABLE,BT_REG_CR1_UDIS);
			Timer_WriteBit(btHandle->instance->CR1,BT_URS_ENABLE,BT_REG_CR1_URS);
			Timer_WriteBit(btHandle->instance->DIER,BT_UIE_ENABLE,BT_REG_DIER_UIE);
			break;
		case BT_MODE_CONT_NO_EVENT:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_DISABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_DISABLE,BT_REG_CR1_UDIS);
			break;
		case BT_MODE_CONT_EVENT_OF:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_DISABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_ENABLE,BT_REG_CR1_UDIS);
			Timer_WriteBit(btHandle->instance->CR1,BT_URS_DISABLE,BT_REG_CR1_URS);
			break;
		case BT_MODE_CONT_EVENT_UG:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_DISABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_ENABLE,BT_REG_CR1_UDIS);
			Timer_WriteBit(btHandle->instance->CR1,BT_URS_DISABLE,BT_REG_CR1_URS);
			break;
		case BT_MODE_CONT_INTERRUPT_OF:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_DISABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_ENABLE,BT_REG_CR1_UDIS);
			Timer_WriteBit(btHandle->instance->CR1,BT_URS_DISABLE,BT_REG_CR1_URS);
			Timer_WriteBit(btHandle->instance->DIER,BT_UIE_ENABLE,BT_REG_DIER_UIE);
			break;
		case BT_MODE_CONT_INTERRUPT_UG:
			Timer_WriteBit(btHandle->instance->CR1,BT_OPM_DISABLE, BT_REG_CR1_OPM);
			Timer_WriteBit(btHandle->instance->CR1,BT_UDIS_ENABLE,BT_REG_CR1_UDIS);
			Timer_WriteBit(btHandle->instance->CR1,BT_URS_ENABLE,BT_REG_CR1_URS);
			Timer_WriteBit(btHandle->instance->DIER,BT_UIE_ENABLE,BT_REG_DIER_UIE);
			break;
		}
		/*	Buffer in the preload for ARR */
		Timer_WriteBit(btHandle->instance->CR1,btHandle->init->Preload_ARR,BT_REG_CR1_ARPE);

		/* prescalar conf. & period To Initialize ARR*/
		do{

			btHandle->init->prescalar = i-1;
			btHandle->init->ARR_Value = BT_PeriodCalculator(btHandle,btHandle->init->period,btHandle->init->CountStartValue);
			i++;
		}
		while((btHandle->init->ARR_Value >= 0xFFFF)||(btHandle->init->ARR_Value <= 0x0000));

		btHandle->instance->PSC = btHandle->init->prescalar;
		btHandle->instance->ARR = btHandle->init->ARR_Value;
		/* set state to In initialized */
		btHandle->state = BT_STATE_INITIALIZED;


	}
}







































