/*
 * stm32f407xx_uart_driver.h
 *
 *  Created on: Feb 3, 2019
 *      Author: Ra2ouf
 */

#ifndef STM32F407XX_UART_DRIVER_H_
#define STM32F407XX_UART_DRIVER_H_

/* Includes */
#include "stm32f4xx.h"
/******************************************************************************/
/*                                                                            */
/*                 1. Universal Asynchronous Receiver Transmitter             */
/*                           Register Bit Defininitions                       */
/******************************************************************************/

/*******************  Bit definition for UART_CR1 register  ********************/
//send break bit to break characters
#define UART_REG_CR1_SENDBREAK	((uint32_t)0U)
#define UART_SENDBREAK_OFF		(0U)
#define UART_SENDBREAK_ON		(1U)
//Receiver WaKe up bit determines if the USART is in mute mode or not. It is set and cleared by software and can be cleared by hardware when a wakeup sequence is recognized.
#define UART_REG_CR1_RWK		((uint32_t)1U)
#define	UART_RWK_R_ACTIVE		(0U)
#define	UART_RWK_R_MUTE			(1U)
//Receiver Enable
#define UART_REG_CR1_RE			((uint32_t)2U)
#define UART_RE_OFF				(0U)
#define UART_RE_ON				(1U)
//Transmitter Enable
#define UART_REG_CR1_TE			((uint32_t)3U)
#define UART_TE_OFF				(0U)
#define UART_TE_ON				(1U)
//IDLE INT. Enable
#define UART_REG_CR1_IDLEIE		((uint32_t)4U)
#define UART_IDLEIE_OFF			(0U)
#define UART_IDLEIE_ON			(1U)
//RXNE interrupt enable
#define UART_REG_CR1_RXNEIE		((uint32_t)5U)
#define UART_RXNEIE_OFF			(0U)
#define UART_RXNEIE_ON			(1U)
//Transmission complete interrupt enable
#define UART_REG_CR1_TCIE		((uint32_t)6U)
#define UART_TCIE_OFF			(0U)
#define UART_TCIE_ON			(1U)
//TXE interrupt e	nable
#define UART_REG_CR1_TXEIE		((uint32_t)7U)
#define UART_TXEIE_OFF			(0U)
#define UART_TXEIE_ON			(1U)
//ParityError interrupt enable
#define UART_REG_CR1_PEIE		((uint32_t)8U)
#define UART_PEIE_OFF			(0U)
#define UART_PEIE_ON			(1U)
//Parity selection
#define UART_REG_CR1_PS			((uint32_t)9U)
#define UART_PS_EVEN			(0U)
#define UART_PS_ODD				(1U)
//Parity control enable ( ON make you put parity on MSB)
#define UART_REG_CR1_PCE		((uint32_t)10U)
#define UART_PCE_OFF			(0U)
#define UART_PCE_ON				(1U)
//Wakeup method bit determines the USART wakeup method, it is set or cleared by software.
#define UART_REG_CR1_WAKE		((uint32_t)11U)
#define UART_WAKE_IDLE_LINE		(0U)
#define UART_WAKE_ADD_MARK		(1U)
//Word length
#define UART_REG_CR1_WORD_LEN	((uint32_t)12U)
#define UART_WORD_LEN_8BIT		(0U)
#define UART_WORD_LEN_9BIT		(1U)
//USART enable
#define UART_REG_CR1_UE			((uint32_t)13U)
#define UART_UE_OFF				(0U)
#define UART_UE_ON				(1U)
/* bit 14 is reserved */
//Oversampling mode
#define UART_REG_CR1_OVER8		((uint32_t)15U)
#define UART_OVER8_16			(0U)
#define UART_OVER8_8			(1U)

/*******************  Bit definition for SPI_CR2 register  ********************/
//Clock phase (USART)
#define UART_REG_CR2_CPHA		((uint32_t)9U)
#define UART_CPHA_FIRST			(0U)
#define UART_CPHA_SECOND		(1U)
//Clock polarity (USART)
#define UART_REG_CR2_CPOL		((uint32_t)10U)
#define UART_CPOL_LOW			(0U)
#define UART_CPOL_HIGH			(1U)
//Clock enable (USART)
#define UART_REG_CR2_CLKEN		((uint32_t)11U)
#define UART_CLKEN_OFF			(0U)
#define UART_CLKEN_ON			(1U)
//STOP bits
#define UART_REG_CR2_STOP		((uint32_t)12U)
#define UART_STOP_1				(0U)
#define UART_STOP_0_5			(1U)
#define UART_STOP_2				(2U)
#define UART_STOP_1_5			(3U)

/*******************  Bit definition for UART_CR2 register  ********************/
//ERROR INT
#define UART_REG_CR3_EIE		((uint32_t)0U)
#define UART_EIE_OFF			(0U)
#define UART_EIE_ON				(1U)

/*******************  Bit definition for UART_SR register  ********************/
//Parity error
#define UART_REG_SR_PE_FLAG		((uint32_t)0U)
//Framing error
#define UART_REG_SR_FE_FLAG		((uint32_t)1U)
//Noise detected flag
#define UART_REG_SR_NE_FLAG		((uint32_t)2U)
//Overrun error
#define UART_REG_SR_ORE_FLAG	((uint32_t)3U)
//IDLE line detected
#define UART_REG_SR_IDLE_FLAG	((uint32_t)4U)
//Read data register not empty
#define UART_REG_SR_RXNE_FLAG	((uint32_t)5U)
//Transmission complete
#define UART_REG_SR_TC_FLAG		((uint32_t)6U)
//Transmit data register empty
#define UART_REG_SR_TXE_FLAG	((uint32_t)7U)
//CTS flag
#define UART_REG_SR_CTS_FLAG	((uint32_t)9U)
/* CONSTANTS */
#define ENABLE		(1U)
#define DISABLE 	(0U)

#define RESET		(0U)
#define SET			(!RESET)

/* Naming UARTS */

#define UART_1		USART1
#define UART_2		USART2
#define UART_3		USART3
#define UART_4		UART4
#define UART_5		UART5
#define UART_6		USART6
#define UART_7		UART7
#define UART_8		UART8

/******************************************************************************/
/*                                                                            */
/*                				 2. DATA Structures         			      */
/*                           							                      */
/******************************************************************************/
//INIT CONSTANTS
enum
{
	UART_MODE_TX	= 0x01,
	UART_MODE_RX	= 0x02,
	UART_MODE_TX_RX	= 0x03
};

enum
{
	UART_WORDLENGTH_8_BIT = UART_WORD_LEN_8BIT,
	UART_WORDLENGTH_9_BIT = UART_WORD_LEN_9BIT
};
enum
{
	UART_STOPBITS_1 	= UART_STOP_1,
	UART_STOPBITS_0_5 	= UART_STOP_0_5,
	UART_STOPBITS_2 	= UART_STOP_2,
	UART_STOPBITS_1_5	= UART_STOP_1_5
};
enum
{
	UART_PARITY_0FF 	= 0x00,
	UART_PARITY_EVEN 	= 0x01,
	UART_PARITY_ODD		= 0x02
};
enum
{
	UART_OVER_SAMPLING_16 = UART_OVER8_16,
	UART_OVER_SAMPLING_8  = UART_OVER8_8
};
//Structures
typedef enum
{
	UART_ERROR_NONE	  = 0x00U,		/*!< No error            */
	UART_ERROR_PE  	  = 0x01U,		/*!< Parity error        */
	UART_ERROR_NE     = 0x02U,		/*!< Noise error         */
	UART_ERROR_FE     = 0x04U,		/*!< Frame error         */
	UART_ERROR_ORE    = 0x08U,		/*!< Overrun error       */
	UART_ERROR_DMA    = 0x10U		/*!< DMA transfer error  */
}tUART_Error;

typedef enum
{
	UART_STATE_RESET   	  = 0x00U,
	UART_STATE_READY  	  = 0x01U,
	UART_STATE_BUSY	   	  = 0x02U,
	UART_STATE_BUSY_TX 	  = 0x12U,
	UART_STATE_BUSY_RX	  = 0x22U,
	UART_STATE_BUSY_TX_RX = 0x32U
}tUART_state;

typedef struct
{
	uint32_t Mode;				/*  Specifies whether the Receive or Transmit mode is enabled or disabled */
	uint32_t WordLength;		/* Specifies the number of data bits transmitted or received in a frame */
	uint32_t StopBits;			/* Specifies the number of stop bits transmitted */
	uint32_t Parity;			/* Specifies the parity mode. */
	uint32_t OverSampling;		/*  Specifies whether the Over sampling 8 is enabled or disabled */
	uint32_t BaudRate;			/* This member configures the UART communication baud rate */
}tUART_Init;

typedef void (tTX_COMP_CB)(void *ptr);
typedef void (tRX_COMP_CB)(void *ptr);
typedef void (tError_CB)(void *ptr);

typedef struct
{
	USART_TypeDef *Instance;
	tUART_Init    *Init;
	uint8_t		  *pTxBuffPtr;
	uint32_t	  TxBuffSize;
	uint32_t	  TxBuffCount;
	uint8_t		  *pRxBuffPtr;
	uint32_t	  RxBuffSize;
	uint32_t	  RxBuffCount;
	tUART_state	  rx_state;
	tUART_state	  tx_state;
	tUART_Error	  error;
	tTX_COMP_CB	  *tx_comp_cb;
	tRX_COMP_CB	  *rx_comp_cb;
	tError_CB	  *error_cb;
}tUART_Config;



/******************************************************************************/
/*                                                                            */
/*                				 3. Exposed APIS	         			      */
/*                           							                      */
/******************************************************************************/
/* Public macro */

//Public helper macros

#define UART_WriteReg(REG,DATA,BIT)	 			((REG) = (((REG)&~(1U<<(BIT)))|((DATA)<<(BIT))))
#define UART_WriteRegMask(REG,DATA,BIT,MASK)	((REG) =( ((REG)&~((MASK)<<BIT))|((DATA)<<BIT)))
#define UART_ReadReg(REG,BIT)		 			(((REG)&(1<<BIT))>>(BIT))
#define UART_ReadRegMASK(REG,BIT,MASK)		 	(((REG)&((MASK)<<(BIT)))>>(BIT))

// Clock Control

#define UART1_Clk_Enable()	(RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#define UART2_Clk_Enable()	(RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define UART3_Clk_Enable()	(RCC->APB1ENR |= RCC_APB1ENR_USART3EN)
#define UART4_Clk_Enable()	(RCC->APB1ENR |= RCC_APB1ENR_UART4EN)
#define UART5_Clk_Enable()	(RCC->APB1ENR |= RCC_APB1ENR_UART5EN)
#define UART6_Clk_Enable()	(RCC->APB2ENR |= RCC_APB2ENR_USART6EN)
#define UART7_Clk_Enable()	(RCC->APB1ENR |= RCC_APB1ENR_UART7EN)
#define UART8_Clk_Enable()	(RCC->APB1ENR |= RCC_APB1ENR_UART8EN)


//Control Macros

#define UART_EorDTx(STATUS, UART_STRUCT)		((STATUS)?(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_TE_ON,UART_REG_CR1_TE)):(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_TE_OFF,UART_REG_CR1_TE)))
#define UART_EorDRx(STATUS, UART_STRUCT)		((STATUS)?(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_RE_ON,UART_REG_CR1_RE)):(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_RE_OFF,UART_REG_CR1_RE)))
#define UART_EorDUART(STATUS, UART_STRUCT)		((STATUS)?(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_UE_ON,UART_REG_CR1_UE)):(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_UE_OFF,UART_REG_CR1_UE)))
#define UART_EorDIntTXE(STATUS, UART_STRUCT)	((STATUS)?(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_TXEIE_ON,UART_REG_CR1_TXEIE)):(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_TXEIE_OFF,UART_REG_CR1_TXEIE)))
#define UART_EorDIntRXNE(STATUS, UART_STRUCT)	((STATUS)?(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_RXNEIE_ON,UART_REG_CR1_RXNEIE)):(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_RXNEIE_OFF,UART_REG_CR1_RXNEIE)))
#define UART_EorDIntTC(STATUS, UART_STRUCT)		((STATUS)?(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_TCIE_ON,UART_REG_CR1_TCIE)):(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_TCIE_OFF,UART_REG_CR1_TCIE)))
#define UART_EorDIntPE(STATUS, UART_STRUCT)		((STATUS)?(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_PEIE_ON,UART_REG_CR1_PEIE)):(UART_WriteReg(UART_STRUCT->Instance->CR1,UART_PEIE_OFF,UART_REG_CR1_PEIE)))
#define UART_EorDIntError(STATUS, UART_STRUCT)	((STATUS)?(UART_WriteReg(UART_STRUCT->Instance->CR3,UART_EIE_ON,UART_REG_CR3_EIE)):(UART_WriteReg(UART_STRUCT->Instance->CR3,UART_EIE_OFF,UART_REG_CR3_EIE)))

//baudrate calculation

#define UART_BAUDRATE(BAUDRATE)		((SystemCoreClock)/(8.0*(2-(uart_handle->Init->OverSampling))*(BAUDRATE)))

// UART INT ENABLE

#define UART_INT_ENABLE(UARTx_IRQ)	(NVIC_EnableIRQ(UARTx_IRQ))

/* Public function prototypes */
void UART_Init(tUART_Config *uart_handle);
void UART_Send(tUART_Config *uart_handle,uint8_t *buffer, uint32_t len);
void UART_Receive(tUART_Config *uart_handle,uint8_t *buffer, uint32_t len);
void UART_IntHandle(tUART_Config *huart);
void UART_ErrorCB(void *huart);
#endif /* STM32F407XX_UART_DRIVER_H_ */
