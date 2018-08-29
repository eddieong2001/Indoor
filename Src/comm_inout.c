/**
 ******************************************************************************
 * File Name          : comm_inout.c
 * Description        : This file provide the customize config to uart1 for communcation between in-door and out-door machine
 ******************************************************************************
 *  Write based on HAL_UART and HAL_DMA APIs.
 *
 *  Set UART1 RX with DMA circular mode for receiving data in the background by HW.
 *  Set UART1 TX with DMA normal mode for sending data when needed.
 *
 *  No use interrupt, use DMA depth register info to manipulate circular buffer operation.
 *  This can reduce system overhead.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "comm_inout.h"

/* #define DBG_INFO_DISPLAY */


/* use 2-level Macro to replace these defines */
/* ------------------------------------------- */
/* #define COMMPC_TX_DMA DMA1_Channel4 */
/* #define COMMPC_RX_DMA DMA1_Channel5 */

/* #define COMMPC_TX_DMA_IRQN DMA1_Channel4_IRQn */
/* #define COMMPC_RX_DMA_IRQN DMA1_Channel5_IRQn */

/*
 * 2-level Macro
 * For example:
 * DMA_CH_ADDR(COMMPC_TX_DMA_IDX, COMMPC_TX_DMA_CHANNEL)
 * -> DMA_CH_ADDR_DEF(1, 4)
 * -> (DMA1_Channel4)
 */
#define DMA_CH_ADDR_DEF(IDX, CH) (DMA##IDX##_Channel##CH)
#define DMA_CH_ADDR(IDX, CH) DMA_CH_ADDR_DEF(IDX, CH)

#if (COMMIO_TX_DMA_CHANNEL==1)
#define DMA_TX_IRQN_DEF(IDX) (DMA##IDX##_Channel1_IRQn)
#elif (COMMIO_TX_DMA_CHANNEL==2 || COMMIO_TX_DMA_CHANNEL==3)
#define DMA_TX_IRQN_DEF(IDX) (DMA##IDX##_Channel2_3_IRQn)
#elif (COMMIO_TX_DMA_CHANNEL==4 || COMMIO_TX_DMA_CHANNEL==5)
#define DMA_TX_IRQN_DEF(IDX) (DMA##IDX##_Channel4_5_IRQn)
#endif

#if (COMMIO_RX_DMA_CHANNEL==1)
#define DMA_RX_IRQN_DEF(IDX) (DMA##IDX##_Channel1_IRQn)
#elif (COMMIO_RX_DMA_CHANNEL==2 || COMMIO_RX_DMA_CHANNEL==3)
#define DMA_RX_IRQN_DEF(IDX) (DMA##IDX##_Channel2_3_IRQn)
#elif (COMMIO_RX_DMA_CHANNEL==4 || COMMIO_RX_DMA_CHANNEL==5)
#define DMA_RX_IRQN_DEF(IDX) (DMA##IDX##_Channel4_5_IRQn)
#endif

#define DMA_TX_IRQN(IDX) DMA_TX_IRQN_DEF(IDX)
#define DMA_RX_IRQN(IDX) DMA_RX_IRQN_DEF(IDX)

#define COMMIO_TX_DMA DMA_CH_ADDR(COMMIO_TX_DMA_IDX, COMMIO_TX_DMA_CHANNEL)
#define COMMIO_RX_DMA DMA_CH_ADDR(COMMIO_RX_DMA_IDX, COMMIO_RX_DMA_CHANNEL)

#define COMMIO_TX_DMA_IRQN DMA_TX_IRQN(COMMIO_TX_DMA_IDX)
#define COMMIO_RX_DMA_IRQN DMA_RX_IRQN(COMMIO_RX_DMA_IDX)


#ifdef DBG_INFO_DISPLAY

extern void UART_PRINT(const char * format, ... );
/* #define DBG_INFO(...) UART_PRINT(__VA_ARGS__) */
#define DBG_INFO(str, args...) UART_PRINT(str, ##args)

#else

#define DBG_INFO(str, args...)

#endif


#define ABR_SYNC_WORD (0x5555) //corresponds to ABR mode3
/* #define DEFAULT_BRR (0xEA60) //corresponds to 1200Bits/sec @ 72M */
#define DEFAULT_BRR (0x9C40) //corresponds to 1200Bits/sec @ 48M


#define REG_COMMIO_CR1 (COMMIO_UART->CR1)
#define REG_COMMIO_CR3 (COMMIO_UART->CR3)
#define REG_COMMIO_BRR (COMMIO_UART->BRR)
#define REG_COMMIO_ISR (COMMIO_UART->ISR)
#define REG_COMMIO_ICR (COMMIO_UART->ICR)
#define REG_COMMIO_TDR (COMMIO_UART->TDR)
#define REG_COMMIO_RDR (COMMIO_UART->RDR)

#define REG_COMMIO_TXDMA_CCR (COMMIO_TX_DMA->CCR)
#define REG_COMMIO_TXDMA_CNDTR (COMMIO_TX_DMA->CNDTR)
#define REG_COMMIO_TXDMA_CPAR (COMMIO_TX_DMA->CPAR)
#define REG_COMMIO_TXDMA_CMAR (COMMIO_TX_DMA->CMAR)

#define REG_COMMIO_RXDMA_CCR (COMMIO_RX_DMA->CCR)
#define REG_COMMIO_RXDMA_CNDTR (COMMIO_RX_DMA->CNDTR)
#define REG_COMMIO_RXDMA_CPAR (COMMIO_RX_DMA->CPAR)
#define REG_COMMIO_RXDMA_CMAR (COMMIO_RX_DMA->CMAR)


#ifdef ABR_SYNC_EVERYTIME

#define REG_COMMIO_RQR (COMMIO_UART->RQR)

#endif


#ifdef DATA_CRC_CHECK

#define REG_CRC_DR (CRC->DR)
#define REG_CRC_CR (CRC->CR)

#endif

extern UART_HandleTypeDef COMMIO_UART_HANDLE;

static ST_COMMIO_DATA stRxData;


HAL_StatusTypeDef Drv_CommIO_Init(void)
{

#ifdef UART_PARITY_CHECK_ON
    /* re-init UART setting to set parity check on */

    if (HAL_OK != HAL_UART_DeInit(&COMMIO_UART_HANDLE)) {
        return HAL_ERROR;
    }

    COMMIO_UART_HANDLE.Init.WordLength = UART_WORDLENGTH_9B;
    COMMIO_UART_HANDLE.Init.Parity = UART_PARITY_EVEN;

    if (HAL_OK != HAL_UART_Init(&COMMIO_UART_HANDLE)) {
        return HAL_ERROR;
    }
#endif

    /* disable IRQ which are enabled by CubeMX created code by default */
    /* reduce IRQ use to save CPU resource */
    HAL_NVIC_DisableIRQ(COMMIO_TX_DMA_IRQN);
    HAL_NVIC_DisableIRQ(COMMIO_RX_DMA_IRQN);

    /* pre-set TXDMA settings, to reduce the cycle in write API */
    REG_COMMIO_TXDMA_CPAR = (uint32_t)&(REG_COMMIO_TDR);

    /* pre-set RXDMA settings, to reduce the cycle in write API */
    REG_COMMIO_RXDMA_CPAR = (uint32_t)&(REG_COMMIO_RDR);

    /* disable RE and UE first */
    REG_COMMIO_CR1 &= ~(USART_CR1_RE | USART_CR1_UE);

    /* set BRR to 1200 bits/sec first */
    REG_COMMIO_BRR = DEFAULT_BRR;

    /* pre-set DMAT and DMAR, since the actual DMA will be triggered by DMA_EN */
    /* set overrun disable, to prevent un-predicable error */
    REG_COMMIO_CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS);

    /* enable UE */
    REG_COMMIO_CR1 |= USART_CR1_UE;

    return HAL_OK;
}

HAL_StatusTypeDef Drv_CommIO_RxRead(ST_COMMIO_DATA **pstRxData)
{
    /* Check that a Rx process is complete or not */
    if (0x0 != REG_COMMIO_RXDMA_CNDTR ) {
        /* if CNDTR is not zero, means the DMA transfer is not complete */
        return HAL_BUSY;
    }

#ifdef UART_PARITY_CHECK_ON
    /* check parity status */
    /* if (gpUartRx->Instance->ISR & 0x1) { */
    /* gpUartRx->Instance->ICR = 0x1; */
    if (REG_COMMIO_ISR & USART_ISR_PE) {
        REG_COMMIO_ICR = USART_ICR_PECF;

        return HAL_ERROR;
    }
#endif

#ifdef DATA_CRC_CHECK
    /* calculate CRC from the received data */
    DATA_CRC_CAL(&stRxData);   
    /* compared */
    if ((uint16_t)REG_CRC_DR != stRxData.u16CRC) {
        /* DBG_INFO("RxRead CRC check error, Cal = 0x%x, Ori = 0x%x\r\n", hcrc.Instance->DR, pstReceivedData->u16CRC); */
        DBG_INFO("RxRead CRC check error, Cal = 0x%x, Ori = 0x%x\r\n", REG_CRC_DR, stRxData.u16CRC);

        return HAL_ERROR;
    }

#endif

    *pstRxData = &stRxData;

    return HAL_OK;
}

HAL_StatusTypeDef Drv_CommIO_TxSend(ST_COMMIO_DATA *pstTxData)
{
    /* to many redundant configuration for our case */
    /* return HAL_UART_Transmit_DMA(gpUart, (uint8_t*)pstTxData, sizeof(ST_COMMIO_DATA)); */

    /* re-write an more efficiency way */

    /* clear Rx RE bit befor sending */
    /* to avoid RX receive the dirty data when TX is sending on the same wire */
    /* CLEAR_BIT(gpUartRx->Instance->CR1, USART_CR1_RE); */
    REG_COMMIO_CR1 &= ~USART_CR1_RE/* ~(USART_CR1_RE | USART_CR1_UE) */;

    /* reset BRR to default value */
    REG_COMMIO_BRR = DEFAULT_BRR;

#ifdef ABR_SYNC_EVERYTIME
    /* set sync word in API, no need user to set manually */
    pstTxData->u16ABRSync = ABR_SYNC_WORD;
#endif

#ifdef DATA_CRC_CHECK
    /* reset CRC at first */
    REG_CRC_CR = CRC_CR_RESET;
    /* cal CRC */
    DATA_CRC_CAL(pstTxData);
    /* set CRC into data */
    pstTxData->u16CRC = (uint16_t)REG_CRC_DR;
    DBG_INFO("CRC = 0x%x\r\n", pstTxData->u16CRC);
    //UART_PRINT("pstTxData.u16CRC = 0x%x\r\n", pstTxData->u16CRC);
#endif

    /* Check that a Tx process is not already ongoing */
    if (0x0 == (REG_COMMIO_ISR & USART_ISR_TC) ) {
        /* if TC_BIT is low, means the last transmission is not complete */
        return HAL_BUSY;
    } else {

        REG_COMMIO_TXDMA_CCR &= ~DMA_CCR_EN;

        /* Configure DMA Channel data length */
        /* gpUartTx->hdmatx->Instance->CNDTR = sizeof(ST_COMMIO_DATA); */
        REG_COMMIO_TXDMA_CNDTR = sizeof(ST_COMMIO_DATA);

        /* Configure DMA Channel source address */
        /* gpUartTx->hdmatx->Instance->CMAR = (uint32_t)pstTxData; */
        REG_COMMIO_TXDMA_CMAR = (uint32_t)pstTxData;

        /* Configure DMA Channel destination address */
        /* pre-set in init func already */
        /* gpUart->hdmatx->Instance->CPAR = (uint32_t)&(gpUart->Instance->TDR); */

        /* enable DMA */
        REG_COMMIO_TXDMA_CCR |= DMA_CCR_EN;

        /* clear TC_BIT */
        REG_COMMIO_ICR = USART_ICR_TCCF;

        /* Enable the DMA transfer for transmit request by setting the DMAT bit
           in the UART CR3 register */
        /* pre-set in init func already */
        /* REG_COMMIO_CR3 |= USART_CR3_DMAT; */

        /* enable UE bit */
        /* pre-set in init function already */
        /* REG_COMMIO_CR1 |= USART_CR1_UE; */

        return HAL_OK;
    }

}

HAL_StatusTypeDef Drv_CommIO_EnableRx(EN_RDEN_MODE enMode)
{
    if (EN_Block == enMode) {

        /* wait until TC flag is set */
        /* while(__HAL_UART_GET_FLAG(gpUartTx, UART_FLAG_TC) == RESET) */
        while((REG_COMMIO_ISR & UART_FLAG_TC) == RESET)
        {}

    } else {

        /* if (__HAL_UART_GET_FLAG(gpUartTx, UART_FLAG_TC) == RESET) */
        if ((REG_COMMIO_ISR & UART_FLAG_TC) == RESET)
        {
            return HAL_BUSY;
        }

    }

#ifdef ABR_SYNC_EVERYTIME
    /* re-init UART-RX to receive ABR sync word again */
    /* __HAL_UART_DISABLE(gpUartRx); */
    /* __HAL_UART_ENABLE(gpUartRx); */

    /* replace restart ABR sync by set RQR with UART_AUTOBAUD_REQUEST */
    /* reset to do measurement again */
    /* gpUartRx->Instance->RQR |= UART_AUTOBAUD_REQUEST; */
    REG_COMMIO_RQR = USART_RQR_ABRRQ;
#endif

#ifdef DATA_CRC_CHECK
    /* reset CRC previously for later check in read API */
    /* hcrc.Instance->CR = 0x1; */
    REG_CRC_CR = CRC_CR_RESET;
#endif

    /* reset RXDMA setting in normal mode */

    /* disable DMA first */
    REG_COMMIO_RXDMA_CCR &= ~DMA_CCR_EN;

    /* Configure DMA Channel data length */
    REG_COMMIO_RXDMA_CNDTR = sizeof(ST_COMMIO_DATA);

    /* Configure DMA Channel destination address */
    REG_COMMIO_RXDMA_CMAR = (uint32_t)(&stRxData);

    /* Configure DMA Channel source address */
    /* pre-set in init func already */
    /* gpUart->hdmarx->Instance->CPAR = (uint32_t)&(gpUart->Instance->RDR); */

    /* enable DMA */
    REG_COMMIO_RXDMA_CCR |= DMA_CCR_EN;

    /* Enable the DMA transfer for receive request by setting the DMAR bit
       in the UART CR3 register */
    /* pre-set in init func already */
    /* REG_COMMIO_CR3 |= USART_CR3_DMAR; */

    /* enable Rx RE bit after transmit complete */
    REG_COMMIO_CR1 |= USART_CR1_RE;

    return HAL_OK;
}

