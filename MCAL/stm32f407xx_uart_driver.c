/*
 * stm32f407xx_uart_driver.c
 *
 *  Created on: Feb 3, 2019
 *      Author: Ra2ouf
 */
#include "stm32f407xx_uart_driver.h"

/* Private macro */

/* Private variables */

/* Private function prototypes */
static void UART_ClearErrorFlag(tUART_Config *huart);
static void UART_IntTxHandle(tUART_Config *huart);
static void UART_IntRxHandle(tUART_Config *huart);
static void UART_IntTCHandle(tUART_Config *huart);
/* Private functions */
static void UART_ClearErrorFlag(tUART_Config *huart)
{
	uint32_t tmpreg = 0U;
	tmpreg = huart->Instance->SR;
	tmpreg = huart->Instance->DR;
}

static void UART_IntTxHandle(tUART_Config *huart)
{
	if(UART_STATE_BUSY_TX == huart->tx_state)
	{
		huart->Instance->DR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);
		if(0U == --huart->TxBuffCount)
		{
			UART_EorDIntTXE(DISABLE, huart);
			UART_EorDIntTC(ENABLE, huart);
		}
		else {/* do nothing */}
	}
	else {/* do nothing */}

}

static void UART_IntRxHandle(tUART_Config *huart)
{
	if(UART_STATE_BUSY_RX == huart->rx_state)
	{
		if(UART_PARITY_0FF == huart->Init->Parity)
		{
			*huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);

		}
		else
		{
			//CHECK FOR WORDLENGTH AND modify for 8bit & 9 bit
			*huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x007F);
		}
		if(0U == --huart->RxBuffCount)
		{
			UART_EorDIntRXNE(DISABLE, huart);
			UART_EorDIntPE(DISABLE, huart);
			UART_EorDIntError(DISABLE, huart);
			huart->rx_state = UART_STATE_READY;
			if(huart->rx_comp_cb)
			{
				huart->rx_comp_cb(&huart->RxBuffSize);
			}
		}
	}
}

static void UART_IntTCHandle(tUART_Config *huart)
{
	UART_EorDIntTC(DISABLE, huart);
	huart->tx_state = UART_STATE_READY;
	if(huart->tx_comp_cb)
	{
		huart->tx_comp_cb(&huart->TxBuffSize);
	}
}

/* Public functions */
void UART_Init(tUART_Config *uart_handle)
{
	/* Initialize Mode */

	if(UART_MODE_TX == uart_handle->Init->Mode)
	{
		UART_WriteReg(uart_handle->Instance->CR1, uart_handle->Init->Mode,UART_REG_CR1_TE);
	}
	else if	(UART_MODE_RX == uart_handle->Init->Mode)
	{
		UART_WriteReg(uart_handle->Instance->CR1, uart_handle->Init->Mode,UART_REG_CR1_RE);
	}
	else
	{
		UART_WriteReg(uart_handle->Instance->CR1, uart_handle->Init->Mode,UART_REG_CR1_TE);
		UART_WriteReg(uart_handle->Instance->CR1, uart_handle->Init->Mode,UART_REG_CR1_RE);
	}

	/* config word length*/

	UART_WriteReg(uart_handle->Instance->CR1, uart_handle->Init->WordLength,UART_REG_CR1_WORD_LEN);

	/* config Stop bits */

	UART_WriteReg(uart_handle->Instance->CR2, uart_handle->Init->StopBits,UART_REG_CR2_STOP);

	/* config Parity */

	if(UART_PARITY_ODD == uart_handle->Init->Parity)
	{
		UART_WriteReg(uart_handle->Instance->CR1, UART_PCE_ON, UART_REG_CR1_PCE);
		UART_WriteReg(uart_handle->Instance->CR1, UART_PS_ODD, UART_REG_CR1_PS);
	}
	else if (UART_PARITY_EVEN == uart_handle->Init->Parity)
	{
		UART_WriteReg(uart_handle->Instance->CR1, UART_PCE_ON, UART_REG_CR1_PCE);
		UART_WriteReg(uart_handle->Instance->CR1, UART_PS_EVEN, UART_REG_CR1_PS);
	}
	else{ /* do nothing */

	}

	/* config. baudrate */

	{
		float temp;
		temp = UART_BAUDRATE(uart_handle->Init->BaudRate);
		uart_handle->Instance->BRR = (((uint32_t)temp<<4))|(uint32_t)((temp-(uint32_t)temp)*16);
	}

	/* Enable Uart and initiate states */

	UART_EorDUART(ENABLE, uart_handle);
	uart_handle->tx_state = UART_STATE_READY;
	uart_handle->rx_state = UART_STATE_READY;
	uart_handle->error = UART_ERROR_NONE;

}

void UART_Send(tUART_Config *uart_handle,uint8_t *buffer, uint32_t len)
{

	uart_handle->pTxBuffPtr = buffer;
	uart_handle->TxBuffCount = len;
	uart_handle->TxBuffSize = len;

	uart_handle->tx_state = UART_STATE_BUSY_TX;
	UART_EorDUART(ENABLE, uart_handle);
	UART_EorDIntTXE(ENABLE, uart_handle);

}

void UART_Receive(tUART_Config *uart_handle,uint8_t *buffer, uint32_t len)
{
	uint32_t val;
	uart_handle->pRxBuffPtr = buffer;
	uart_handle->RxBuffCount = len;
	uart_handle->RxBuffSize = len;

	uart_handle->rx_state = UART_STATE_BUSY_RX;
	UART_EorDIntPE(ENABLE, uart_handle);
	UART_EorDIntError(ENABLE, uart_handle);
	val = uart_handle->Instance->DR;
	UART_EorDIntRXNE(ENABLE, uart_handle);

}

void UART_IntHandle(tUART_Config *huart)
{
	uint32_t temp1 = 0U;
	uint32_t temp2 = 0U;

	//PARITY ERROR INT

	temp1 = UART_ReadReg(huart->Instance->SR, UART_REG_SR_PE_FLAG);
	temp2 = UART_ReadReg(huart->Instance->CR1, UART_REG_CR1_PEIE);
	if ((RESET != temp1) && (RESET != temp2))
	{
		huart->error |= UART_ERROR_PE;
		UART_ClearErrorFlag(huart);

	}
	else{ /* do nothing */ }

	//FRAME ERROR INT

	temp1 = UART_ReadReg(huart->Instance->SR, UART_REG_SR_FE_FLAG);
	temp2 = UART_ReadReg(huart->Instance->CR3, UART_REG_CR3_EIE);
	if ((RESET != temp1) && (RESET != temp2))
	{
		huart->error |= UART_ERROR_FE;
		UART_ClearErrorFlag(huart);

	}
	else{ /* do nothing */ }

	//NOISE ERROR INT

	temp1 = UART_ReadReg(huart->Instance->SR, UART_REG_SR_NE_FLAG);
	temp2 = UART_ReadReg(huart->Instance->CR3, UART_REG_CR3_EIE);
	if ((RESET != temp1) && (RESET != temp2))
	{
		huart->error |= UART_ERROR_NE;
		UART_ClearErrorFlag(huart);

	}
	else{ /* do nothing */ }

	//OVER RUN ERROR INT

	temp1 = UART_ReadReg(huart->Instance->SR, UART_REG_SR_ORE_FLAG);
	temp2 = UART_ReadReg(huart->Instance->CR3, UART_REG_CR3_EIE);
	if ((RESET != temp1) && (RESET != temp2))
	{
		huart->error |= UART_ERROR_ORE;
		UART_ClearErrorFlag(huart);

	}
	else{ /* do nothing */ }

	//TRANSMITT DATA INT

	temp1 = UART_ReadReg(huart->Instance->SR, UART_REG_SR_TXE_FLAG);
	temp2 = UART_ReadReg(huart->Instance->CR1, UART_REG_CR1_TXEIE);
	if ((RESET != temp1) && (RESET != temp2))
	{
		UART_IntTxHandle(huart);
		return;
	}
	else{ /* do nothing */ }

	//RECEIVER DATA INT

	temp1 = UART_ReadReg(huart->Instance->SR, UART_REG_SR_RXNE_FLAG);
	temp2 = UART_ReadReg(huart->Instance->CR1, UART_REG_CR1_RXNEIE);
	if ((RESET != temp1) && (RESET != temp2))
	{
		UART_IntRxHandle(huart);
		return;
	}
	else{ /* do nothing */ }

	//TRANSMITTER COMPELETE INT

	temp1 = UART_ReadReg(huart->Instance->SR, UART_REG_SR_TC_FLAG);
	temp2 = UART_ReadReg(huart->Instance->CR1, UART_REG_CR1_TCIE);
	if ((RESET != temp1) && (RESET != temp2))
	{
		UART_IntTCHandle(huart);
		return;
	}
	else{ /* do nothing */ }

	// Call Back function for error

	if(UART_ERROR_NONE != huart->error)
	{
		huart->error_cb(&huart);
	}
}

