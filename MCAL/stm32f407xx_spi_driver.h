#ifndef 	_HAL_STM32f407XX_SPI_DRIVER_H_
#define		_HAL_STM32f407XX_SPI_DRIVER_H_

/* Uc specific header file for stm32407vg based discovery board */
#include "stm32f4xx.h"

/*SPI Slaves Devices number Defination */
#define SPI_SLAVE_DEVICES_NUMBER 	(1)
/******************************************************************************/
/*                                                                            */
/*                        1. Serial Peripheral Interface                      */
/*                           Register Bit Defininitions                       */
/******************************************************************************/

/*******************  Bit definition for SPI_CR1 register  ********************/

#define SPI_REG_CR1_BIDIMODE								((uint32_t)15U)
#define	 SPI_BIDI_MODE_2_LINE_UNI_DIR						(0U)
#define SPI_BIDI_MODE_1_LINE_BI_DIR							(1U)

#define	SPI_REG_CR1_BIDIOE									((uint32_t)14U)
#define	SPI_BIDI_OE_OP_ENABLE								(0U)	/* Receive only  mode*/
#define	SPI_BIDI_OE_OP_DISABLE								(1U)	/* transmit only mode */

#define SPI_REG_CR1_CRCEN									((uint32_t)13U) /* This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation. */
#define SPI_CRCEN_DISABLE									(0U)
#define SPI_CRCEN_ENABLE									(1U)

#define SPI_REG_CR1_CRCNEXT									((uint32_t)12U)
#define	SPI_CRCNEXT_DATA_PHASE								(0U)
#define	SPI_CRCNEXT_NEXT_TRANSFER							(1U)

#define	SPI_REG_CR1_DFF										((uint32_t)11U)
#define SPI_DFF_8_BIT										(0U)
#define SPI_DFF_16_BIT										(1U)

#define SPI_REG_CR1_RXONLY									((uint32_t)10U)
#define SPI_RXONLY_DISABLE									(0U)
#define SPI_RXONLY_ENABLE									(1U)

#define SPI_REG_CR1_SSM										((uint32_t)9U)
#define SPI_SSM_DISABLE										(0U)
#define SPI_SSM_ENABLE										(1U)

#define SPI_REG_CR1_SSI										((uint32_t)8U)
#define SPI_SSI_HIGH										(1U)
#define	SPI_SSI_LOW											(0U)

#define SPI_REG_CR1_LSBFIRST								((uint32_t)7U)
#define SPI_LSBFIRST_MSBFIRST								(0U)
#define SPI_LSBFIRST_LSBFIRST								(1U)

#define SPI_REG_CR1_SPE										((uint32_t)6U)
#define SPI_SPE_DISABLE										(0U)
#define SPI_SPE_ENABLE										(1U)

#define	SPI_REG_CR1_BR										((uint32_t)3U)
#define SPI_BR_PCLK_DIV_2									(0x00U)
#define SPI_BR_PCLK_DIV_4									(0x01U)	
#define SPI_BR_PCLK_DIV_8                				 	(0x02U)
#define SPI_BR_PCLK_DIV_16                					(0x03U)
#define SPI_BR_PCLK_DIV_32                					(0x04U)
#define SPI_BR_PCLK_DIV_64                					(0x05U)
#define SPI_BR_PCLK_DIV_128               					(0x06U)
#define SPI_BR_PCLK_DIV_256               					(0x07U)

#define	SPI_REG_CR1_MSTR									((uint32_t)2U)
#define SPI_MSTR_SLAVE_CONF									(0U)
#define SPI_MSTR_MASTER_CONF								(1U)

#define SPI_REG_CR1_CPOL									((uint32_t)1U)
#define	SPI_CPOL_0_IDLE										(0U)	
#define	SPI_CPOL_1_IDLE                   					(1U)

#define SPI_REG_CR1_CPHASE									((uint32_t)0U)
#define SPI_CPHASE_FIRST_EDGE								(0U)
#define SPI_CPHASE_SECOND_EDGE								(1U)

/*******************  Bit definition for SPI_CR2 register  ********************/

#define SPI_REG_CR2_TXEIE									((uint32_t)7U)
#define SPI_TXEIE_DISABLE									(0U)
#define SPI_TXEIE_ENABLE									(1U)

#define SPI_REG_CR2_RXNEIE									((uint32_t)6U)
#define SPI_RXNEIE_DISABLE									(0U)
#define SPI_RXNEIE_ENABLE 									(1U)

#define SPI_REG_CR2_ERRIE									((uint32_t)5U)
#define SPI_ERRIE_DISABLE									(0U)
#define SPI_ERRIE_ENABLE 									(1U)

#define SPI_REG_CR2_FRF										((uint32_t)4U)
#define SPI_FRF_MOTOROLA_MODE								(0U)
#define SPI_FRF_TI_MODE 									(1U)

#define SPI_REG_CR2_SSOE									((uint32_t)2U)
#define SPI_SSOE_DISABLE_MASTERMODE							(0U)
#define SPI_SSOE_ENABLE_MASTERMODE 							(1U)

#define SPI_REG_CR2_TXDMAEN									((uint32_t)1U)
#define SPI_TXDMAEN_TXDMA_BUFFER_DISABLE					(0U)
#define SPI_TXDMAEN_TXDMA_BUFFER_ENABLE						(1U)

#define SPI_REG_CR2_RXDMAEN									((uint32_t)0U)
#define SPI_RXDMAEN_RXDMA_BUFFER_DISABLE					(0U)
#define SPI_RXDMAEN_RXDMA_BUFFER_ENABLE						(1U)

/*******************  Bit definition for SPI_SR register  ********************/

#define SPI_REG_SR_FRE_FLAG        							((uint32_t)8U)
#define SPI_REG_SR_BUSY_FLAG        						((uint32_t)7U)
#define	SPI_REG_SR_OVR_FLAG									((uint32_t)6U)
#define	SPI_REG_SR_MODF_FLAG								((uint32_t)5U)
#define	SPI_REG_SR_CRCERR_FLAG								((uint32_t)4U)
#define	SPI_REG_SR_UDR_FLAG									((uint32_t)3U)
#define	SPI_REG_SR_CHSIDE_FLAG								((uint32_t)2U)
#define SPI_REG_SR_TXE_FLAG        							((uint32_t)1U)
#define SPI_REG_SR_RXNE_FLAG        						((uint32_t)0U)

/* SPI device base address */

#define SPI_1					SPI1
#define SPI_2					SPI2
#define SPI_3					SPI3

/* CONST. Definations */

#define RESET			(0U)
#define SET				!RESET

#define NULL			(0U)

/* Macros to Enable clock for SPI devices */

#define	_HAL_SPI1_CLK_ENABLE()			((RCC->APB2ENR) |= (1U << 12))
#define	_HAL_SPI2_CLK_ENABLE()			((RCC->APB1ENR) |= (1U << 14))
#define	_HAL_SPI3_CLK_ENABLE()			((RCC->APB1ENR) |= (1U << 15))

/* SPI HELPER MACROS */

#define SPI_WriteReg(REG, DATA, BIT)			( (REG) = ( (((REG) & ~(1U<<(BIT)))|((DATA))<<(BIT))) )
#define SPI_ReadReg(REG, BIT)					( ((REG)&((1U)<<(BIT)))>>(BIT) )

/* MACRO to Enable & Disable SPI*/

#define _HAL_SPI_ENABLE(HANDLE_STRUCT)		(SPI_WriteReg(HANDLE_STRUCT->Instance->CR1, SPI_SPE_ENABLE, SPI_REG_CR1_SPE))
//#define _HAL_SPI_ENABLE()		(spi_handle->Instance->CR1 |= (SPI_SPE_ENABLE<<SPI_REG_CR1_SPE))
//#define _HAL_SPI_DISABLE() 	(spi_handle->Instance->CR1 &= ~(SPI_SPE_ENABLE<<SPI_REG_CR1_SPE))
#define _HAL_SPI_DISABLE(HANDLE_STRUCT) 	(SPI_WriteReg(HANDLE_STRUCT->Instance->CR1, SPI_SPE_DISABLE, SPI_REG_CR1_SPE))

/*MACRO to Enable SPI Interrupts */

///#define	_HAL_SPI_IntTxeEnable()		(spi_handle->Instance->CR2 |= (SPI_TXEIE_ENABLE << SPI_REG_CR2_TXEIE))
#define	_HAL_SPI_IntTxeEnable(HANDLE_STRUCT)		(SPI_WriteReg(HANDLE_STRUCT->Instance->CR2, SPI_TXEIE_ENABLE, SPI_REG_CR2_TXEIE))
///define	_HAL_SPI_IntTxeDisable()	(spi_handle->Instance->CR2 &= ~(SPI_TXEIE_ENABLE << SPI_REG_CR2_TXEIE))
#define	_HAL_SPI_IntTxeDisable(HANDLE_STRUCT)	(SPI_WriteReg(HANDLE_STRUCT->Instance->CR2, SPI_TXEIE_DISABLE, SPI_REG_CR2_TXEIE))

///#define _HAL_SPI_IntRxneEnable()	(spi_handle->Instance->CR2 |= (SPI_RXNEIE_ENABLE << SPI_REG_CR2_RXNEIE))
#define _HAL_SPI_IntRxneEnable(HANDLE_STRUCT)	(SPI_WriteReg(HANDLE_STRUCT->Instance->CR2, SPI_RXNEIE_ENABLE, SPI_REG_CR2_RXNEIE))
///#define _HAL_SPI_IntRxneDisable()	(spi_handle->Instance->CR2 &= ~(SPI_RXNEIE_ENABLE << SPI_REG_CR2_RXNEIE))
#define _HAL_SPI_IntRxneDisable(HANDLE_STRUCT)	(SPI_WriteReg(HANDLE_STRUCT->Instance->CR2, SPI_RXNEIE_DISABLE, SPI_REG_CR2_RXNEIE))

/* MACRO TO READ SPI BUSY FLAG */

#define _HAL_SPI_ReadBusyFlag(HANDLE_STRUCT)	(SPI_ReadReg(HANDLE_STRUCT->Instance->SR, SPI_REG_SR_BUSY_FLAG))

/* Macro to Enable SPI_IRQ */

#define _SPI_INT_ENABLE(SPI_IRQN)	(NVIC_EnableIRQ(SPI_IRQN))

/* Macro to Disable SPI_IRQ */
#define _SPI_INT_DISABLE(SPI_IRQN)	(NVIC_DisableIRQ(SPI_IRQN))

/******************************************************************************/
/*                                                                            */
/*                      2. Data types & Structures used by SPI Driver         */
/*                                                                            */
/******************************************************************************/

/* SPI STATES STRUCTURE */

typedef enum 
{
	HAL_SPI_STATE_RESET 			= 0x00U,
	HAL_SPI_STATE_READY 			= 0x01U,
	HAL_SPI_STATE_BUSY 				= 0x02U,
	HAL_SPI_STATE_BUSY_TX 			= 0x12U,
	HAL_SPI_STATE_BUSY_RX 			= 0x22U,
	HAL_SPI_STATE_BUSY_TX_RX 		= 0x32U,
	HAL_SPI_STATE_ERROR				= 0x03U
}hal_spi_state_t;

/* SPI configuration structure */

typedef struct
{
	uint32_t 	Mode; 							/*  Specifies the SPI operating mode MSTR|SLAVE*/
	uint32_t	Direction;					/*  Specifies the SPI Directional mode state. */
	uint32_t	Datasize;						/*  Specifies the SPI data size. */
	uint32_t	CLKPolarity;				/*  Specifies the serial clock steady state. */
	uint32_t	CLKPhase;						/*  Specifies the clock active edge for the bit capture. */
	uint32_t	NSS;								/*  Specifies whether the NSS signal is managed by
                                    hardware (NSS pin) or by software using the SSI bit. */
	uint32_t	BaudRatePrescalar;	/*  Specifies the Baud Rate prescaler value which will be
                                    used to configure the transmit and receive SCK clock. */
	uint32_t	Firstbit;						/*  Specifies whether data transfers start from MSB or LSB bit. */
}spi_init_t;	

/* Handle Structure defination for SPI */

typedef struct 
{
	SPI_TypeDef					*Instance;			/* SPI registers base address */
	spi_init_t					*Init;						/* SPI communication parameters */
	uint8_t 					*pTxBuffPtr;		/* Pointer to SPI Tx transfer Buffer */
	uint16_t					TxXferSize;
	uint16_t					TxXferCount;		
	uint8_t						*pRxBuffPtr;
	uint16_t					RxXferSize;
	uint16_t					RxXferCount;
	hal_spi_state_t				state;
}spi_handle_t;

/******************************************************************************/
/*                                                                            */
/*                      3. Driver Exposed API																	*/
/*                                                                            */
/******************************************************************************/

/* SPI INITILIZATION API */

void hal_SPI_Init(spi_handle_t *spi_handle);


/* SPI Master TX API */

void hal_SPI_MasterSend(spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len);

/* SPI Master RX API */

void hal_SPI_MasterRecieve(spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len);

/* SPI Slave TX API */

void hal_SPI_SlaveSend(spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len);

/* SPI Slave RX API */

void hal_SPI_SlaveRecieve(spi_handle_t *spi_handle, uint8_t *buffer, uint16_t len);

/* SPI Interrupt Handling API*/

void hal_SPI_IrqHandler(spi_handle_t *hspi);

#endif	//_HAL_STM32f407XX_SPI_DRIVER_H_
