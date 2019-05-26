#include <stdint.h>
#include "stm32f407xx_spi_driver.h"

/****************************************************************************/
/* 									 				1.static helper Functions  											*/
/****************************************************************************/

/******************  static helper Functions Prototypes  ********************/

/* SPI Interrupt Tx Handling API*/

static void hal_SPI_IntTxHandle(spi_handle_t *hspi);

/* SPI Interrupt Rx Handling API*/

static void hal_SPI_IntRxHandle(spi_handle_t *hspi);

/******************  static helper Functions Definations  *******************/

/* SPI Interrupt Tx Handling API*/

static void hal_SPI_IntTxHandle(spi_handle_t *hspi)
{
	if(SPI_DFF_8_BIT == hspi->Init->Datasize )
	{
	    hspi->Instance->DR = (*hspi->pTxBuffPtr++);
		hspi->TxXferCount--; //we sent 1 byte
	}
	else
	{
		hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
		hspi->pTxBuffPtr  += 2;
		hspi->TxXferCount -= 2;
	}
	if(0U == hspi->TxXferCount)
	{
		_HAL_SPI_IntTxeDisable(hspi);
	
		if((SPI_MSTR_MASTER_CONF == hspi->Init->Mode) && (HAL_SPI_STATE_BUSY_RX != hspi->state))
		{
			hspi->state = HAL_SPI_STATE_READY;
		
		}
	}
	
}

/* SPI Interrupt Rx Handling API*/

static void hal_SPI_IntRxHandle(spi_handle_t *hspi)
{
	if( SPI_DFF_8_BIT == hspi->Init->Datasize)
	{
		if(NULL != hspi->pRxBuffPtr)
		{
			*(hspi->pRxBuffPtr) = hspi->Instance->DR;
			hspi->pRxBuffPtr++;
			hspi->RxXferCount--;
			
		}
		else {/* DO NOTHING */}
		
	}
	else
	{
		if(NULL != hspi->pRxBuffPtr)
		{
			*((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
			hspi->pRxBuffPtr += 2;
			hspi->RxXferCount -= 2;
			
		}
	}		
	if(0U == hspi->RxXferCount)
	{
		while(0U != _HAL_SPI_ReadBusyFlag(hspi));
		
		_HAL_SPI_IntRxneDisable(hspi);
		hspi->state = HAL_SPI_STATE_READY;
	}
	
}




/******************************************************************************/
/*                                                                            */
/*                      2. Driver Exposed API	Defination											*/
/*                                                                            */
/******************************************************************************/

/* SPI INITILIZATION API */

void hal_SPI_Init(spi_handle_t *spi_handle)
{
	//Initialize SPI CLOCK for Polarity & Phase
	SPI_WriteReg(spi_handle->Instance->CR1, spi_handle->Init->CLKPolarity, SPI_REG_CR1_CPOL);
	SPI_WriteReg(spi_handle->Instance->CR1, spi_handle->Init->CLKPhase, SPI_REG_CR1_CPHASE);
	
	//Initialize SPI MODE
	SPI_WriteReg(spi_handle->Instance->CR1, spi_handle->Init->Mode, SPI_REG_CR1_MSTR);
	
	//Initialize SPI DataSize
	SPI_WriteReg(spi_handle->Instance->CR1, spi_handle->Init->Datasize, SPI_REG_CR1_DFF);
	SPI_WriteReg(spi_handle->Instance->CR1, spi_handle->Init->Firstbit, SPI_REG_CR1_LSBFIRST);
	
	//Initialize SPI Slave Select	line
	#if (1 == SPI_SLAVE_DEVICES_NUMBER)
		SPI_WriteReg(spi_handle->Instance->CR1, spi_handle->Init->NSS, SPI_REG_CR1_SSM);
		if(SPI_SSM_ENABLE == SPI_ReadReg(spi_handle->Instance->CR1, SPI_REG_CR1_SSM))
		{
			if(SPI_MSTR_MASTER_CONF == SPI_ReadReg(spi_handle->Instance->CR1, SPI_REG_CR1_MSTR))
			{
				SPI_WriteReg(spi_handle->Instance->CR1, SPI_SSI_HIGH, SPI_REG_CR1_SSI);
			}
			else
			{
				SPI_WriteReg(spi_handle->Instance->CR1, SPI_SSI_LOW, SPI_REG_CR1_SSI);			
			}
		}
		else{/* Do Nothing */}
	#elif	(1 < SPI_SLAVE_DEVICES_NUMBER)
		SPI_WriteReg(spi_handle->Instance->CR1, SPI_SSM_DISABLE, SPI_REG_CR1_SSM);
	#endif
	
	//Initialize SPI BaudRate
	if(SPI_BR_PCLK_DIV_256 < spi_handle->Init->BaudRatePrescalar )
	{
		SPI_WriteReg(spi_handle->Instance->CR1, SPI_BR_PCLK_DIV_2, SPI_REG_CR1_BR);
	}
	else
	{
		SPI_WriteReg(spi_handle->Instance->CR1, spi_handle->Init->BaudRatePrescalar, SPI_REG_CR1_BR);
	}
	
	//Initialize SPI Direction
	SPI_WriteReg(spi_handle->Instance->CR1, spi_handle->Init->Direction, SPI_REG_CR1_BIDIMODE);
	
}

/* SPI Master TX API */

void hal_SPI_MasterSend(spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len)
{
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxXferSize = len;
	spi_handle->TxXferCount = len;
	
	spi_handle->RxXferCount = 0;
	spi_handle->RxXferSize = 0;
	
	spi_handle->state = HAL_SPI_STATE_BUSY_TX;
	
	_HAL_SPI_IntTxeEnable(spi_handle);
	_HAL_SPI_ENABLE(spi_handle);

}

/* SPI Master RX API */

uint8_t dummy_tx[2] = {0};
void hal_SPI_MasterRecieve(spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len)
{
	uint32_t val;
	spi_handle->pTxBuffPtr 		= dummy_tx;
	spi_handle->TxXferSize		= len;
	spi_handle->TxXferCount		= len;
	
	spi_handle->pRxBuffPtr 		= buffer;
	spi_handle->RxXferSize		= len;
	spi_handle->RxXferCount		= len;
	
	val = spi_handle->Instance->DR;
	spi_handle->state 			  = HAL_SPI_STATE_BUSY_RX;
	
	_HAL_SPI_IntTxeEnable(spi_handle);
	_HAL_SPI_IntRxneEnable(spi_handle);
	_HAL_SPI_ENABLE(spi_handle);
	
}

/* SPI Slave TX API */

uint8_t dummy_rx[10] = {0};
void hal_SPI_SlaveSend(spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len)
{
	spi_handle->pRxBuffPtr 	= dummy_rx;
	spi_handle->RxXferCount = len;
	spi_handle->RxXferSize	= len;
	
	spi_handle->pTxBuffPtr 	= buffer;
	spi_handle->TxXferCount	= len;
	spi_handle->TxXferSize	= len;
	
	spi_handle->state = HAL_SPI_STATE_BUSY_TX;
	
	_HAL_SPI_IntRxneEnable(spi_handle);
	_HAL_SPI_IntTxeEnable(spi_handle);
	
	_HAL_SPI_ENABLE(spi_handle);
	
}

/* SPI Slave RX API */

void hal_SPI_SlaveRecieve(spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len)
{
	spi_handle->pRxBuffPtr 	= buffer;
	spi_handle->RxXferCount	= len;
	spi_handle->RxXferSize	= len;
	
	spi_handle->state		= HAL_SPI_STATE_BUSY_RX;
	
	_HAL_SPI_IntRxneEnable(spi_handle);
	
	_HAL_SPI_ENABLE(spi_handle);
	
}

/* SPI Interrupt Handling API*/

void hal_SPI_IrqHandler(spi_handle_t *hspi)
{
	volatile uint32_t tmp1;
	volatile uint32_t tmp2;
	///Must RXNE be First to check Bec In Mster recevie it also send dummy byte
	tmp1 = (hspi->Instance->SR & (1U << SPI_REG_SR_RXNE_FLAG));
	tmp2 = (hspi->Instance->CR2 & (SPI_RXNEIE_ENABLE << SPI_REG_CR2_RXNEIE));

	if((RESET != tmp1) && (RESET != tmp2))
	{
		hal_SPI_IntRxHandle(hspi);
		return;

	}
	else
	{/*Do Nothing*/}

	tmp1 = (hspi->Instance->SR & (1U << SPI_REG_SR_TXE_FLAG));
	tmp2 = (hspi->Instance->CR2 & (SPI_TXEIE_ENABLE << SPI_REG_CR2_TXEIE));
	if((RESET != tmp1) && (RESET != tmp2))
	{
		hal_SPI_IntTxHandle(hspi);
		return;

	}

}

