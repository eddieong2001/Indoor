/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMM_INOUT_H
#define __COMM_INOUT_H
/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_hal.h"

#define ABR_SYNC_EVERYTIME
#define DATA_CRC_CHECK
#define UART_PARITY_CHECK_ON

#define COMMIO_UART (USART1)
#define COMMIO_UART_HANDLE (huart1)

#define COMMIO_TX_DMA_IDX 1
#define COMMIO_TX_DMA_CHANNEL 4
#define COMMIO_RX_DMA_IDX 1
#define COMMIO_RX_DMA_CHANNEL 3

#ifdef DATA_CRC_CHECK
#define DATA_CRC_CAL(DATA)\
    do {\
        REG_CRC_DR = (DATA)->u16CmdResp;\
        REG_CRC_DR = (DATA)->u16Param1;\
        REG_CRC_DR = (DATA)->u16Param2;\
        REG_CRC_DR = (DATA)->u16Param3;\
    } while (0);
#endif

typedef struct _ST_COMMIO_DATA {

#ifdef ABR_SYNC_EVERYTIME
    uint16_t u16ABRSync;
#endif
    uint16_t u16CmdResp;
    uint16_t u16Param1;
    uint16_t u16Param2;
    uint16_t u16Param3;
#ifdef DATA_CRC_CHECK
    uint16_t u16CRC;
#endif

} ST_COMMIO_DATA;

typedef enum {
    EN_NonBlock,
    EN_Block
} EN_RDEN_MODE;


HAL_StatusTypeDef Drv_CommIO_Init(void);

/*
 * need to check the return value
 * if return = HAL_BUSY, no valid RX data received from last valid received yet
 * if return = HAL_ERROR, CRC check error or Parity check error if error check is enabled
 * if return = HAL_OK, pstRxData will get the latest data
 */
HAL_StatusTypeDef Drv_CommIO_RxRead(ST_COMMIO_DATA **pstRxData);

/*
 * need to check the return value
 * if return = HAL_BUSY, TX bus is still busy by last transmission
 * if return = HAL_OK, pstTxData is sent successfully
 */
HAL_StatusTypeDef Drv_CommIO_TxSend(ST_COMMIO_DATA *pstTxData);

/*
 * need to set enMode to be block or nonblock
 * block will be blocked in loop until last TX transmit is complete
 * nonblock will return HAL_BUSY immediately if TX transmit is not complete
 * or return HAL_OK if TX transmit is complete
 */
HAL_StatusTypeDef Drv_CommIO_EnableRx(EN_RDEN_MODE enMode);

#endif  /* __COMM_INOUT_H */

