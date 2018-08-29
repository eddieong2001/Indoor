/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H
/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_hal.h"

#define LED_UART (USART3)
#define LED_UART_HANDLE (huart3)

#define LED_TX_DMA (DMA1_Channel2)
#define LED_TX_DMA_IRQN (DMA1_Channel2_3_IRQn)

typedef enum {
    EN_DISPLAY_ON_0 = 0x0, //0 means off
    EN_DISPLAY_ON_1 = 0x8,
    EN_DISPLAY_ON_2 = 0x4,
    EN_DISPLAY_ON_3 = 0xC,
    EN_DISPLAY_ON_4 = 0x2,
    EN_DISPLAY_ON_5 = 0xA,
    EN_DISPLAY_ON_6 = 0x6,
    EN_DISPLAY_ON_7 = 0xE,
    EN_DISPLAY_ON_8 = 0x1,
    EN_DISPLAY_ON_9 = 0x9,
    EN_DISPLAY_ON_10 = 0x5,
    EN_DISPLAY_ON_11 = 0xD,
    EN_DISPLAY_ON_12 = 0x3,
    EN_DISPLAY_ON_13 = 0xB,
    EN_DISPLAY_ON_14 = 0x7,
    EN_DISPLAY_ON_15 = 0xF,
} EN_DISPLAY_ON_PERCENTAGE;

typedef enum {
    EN_DISPLAY_DRIVING_1 = 0x0, // 1/8
    EN_DISPLAY_DRIVING_2 = 0x4, // 2/8
    EN_DISPLAY_DRIVING_3 = 0x2, // 3/8
    EN_DISPLAY_DRIVING_4 = 0x6, // 4/8
    EN_DISPLAY_DRIVING_5 = 0x1, // 5/8
    EN_DISPLAY_DRIVING_6 = 0x5, // 6/8
    EN_DISPLAY_DRIVING_7 = 0x3, // 7/8
    EN_DISPLAY_DRIVING_8 = 0x7, // 8/8
} EN_DISPLAY_DRIVING;

typedef enum {
    EN_DISPLAY_SEG_7 = 0x1,
    EN_DISPLAY_SEG_8 = 0x0,
} EN_DISPLAY_SEGMODE;

void Drv_LED_Init(void);

/*
 * [Note]
 * the time delay between these API called should be at least 3ms
 */

HAL_StatusTypeDef Drv_LED_CtrlSet(EN_DISPLAY_ON_PERCENTAGE enPercent, EN_DISPLAY_DRIVING enDriv, EN_DISPLAY_SEGMODE enSegMode);

HAL_StatusTypeDef Drv_LED_DisplayClear(void);

HAL_StatusTypeDef Drv_LED_NumDisplay(uint8_t num);

HAL_StatusTypeDef Drv_LED_ErrCodeDisplay(uint8_t num);

HAL_StatusTypeDef Drv_LED_ColorLEDCtrl(uint8_t red_on, uint8_t yellow_on, uint8_t green_on);

#endif  /* __LED_H */
