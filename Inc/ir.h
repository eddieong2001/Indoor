/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IR_H
#define __IR_H
/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_hal.h"

#define IR_TIM (TIM15)
#define IR_TIM_HANDLE (htim15)
#define IR_TIM_DMA_HANDLE (hdma_tim15_ch1_up_trig_com)

#define IR_TIM_PWM_IN_FALLING_CH (TIM_CHANNEL_2)
#define IR_TIM_PWM_IN_RISING_CH (TIM_CHANNEL_1)
#define IR_TIM_DMABASE_CCR (TIM_DMABASE_CCR1)
#define IR_TIM_DMASRC (TIM_DMA_CC1)

#define IR_DMA_IDX 1
#define IR_DMA_CHANNEL 5

// the decode thresholds, unit is 0.1ms
#define DECODE_TH_STARTBIT (135)
#define DECODE_TH_STOPBIT (75)
#define DECODE_TH_HIGHBIT (40)

typedef struct _ST_IR_DATA {
    uint8_t set_temp;
    uint8_t body_sens;
    uint8_t curtime_hour;
    uint8_t curtime_min;
    uint8_t curtime_AMPM;
    uint8_t power_en;
    uint8_t sleep_en;
    uint8_t ABCode;
    uint8_t ontime_hour;
    uint8_t ontime_min;
    uint8_t ontime_AMPM;
    uint8_t ontime_en;
    uint8_t offtime_hour;
    uint8_t offtime_min;
    uint8_t offtime_AMPM;
    uint8_t offtime_en;
    uint8_t hour_off_en;
    uint8_t hour_off_hour;
    uint8_t warm_en;
    uint8_t cool_en;
    uint8_t dry_en;
    uint8_t fan_en;
    uint8_t auto_en;
    uint8_t airclean_en;
    uint8_t dir_manual_en;
    uint8_t dir_auto_en;
    uint8_t auto_temp_down;
    uint8_t auto_temp_up;
    uint8_t fan_high_en;
    uint8_t fan_mid_en;
    uint8_t fan_low_en;
    uint8_t fan_auto_en;
} ST_IR_DATA;


// typedef void (*IR_CBFunc) (void);


void Drv_IR_Init(void);

void Drv_IR_Receive_DMA_Start(void);

void Drv_IR_Receive_DMA_Stop(void);

HAL_StatusTypeDef Drv_IR_GetData(ST_IR_DATA **ppstData);

void Drv_IR_Error_Recover(void);

// HAL_StatusTypeDef Drv_IR_CB_Register(IR_CBFunc cb);

#endif  /* __IR_H */
