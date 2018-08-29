/**
 ******************************************************************************
 * File Name          : ir.c
 * Description        : This file provide the customize config to timer for indoor IR receive handle
 ******************************************************************************
 *  Write based on HAL_TIM and HAL_DMA APIs.
 *
 *  Set timer to PWM input mode with DMA triggered to receive IR pulse data.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "ir.h"

/* #define DBG_INFO_DISPLAY */

#define TICK_RECOVER_CHECK
#define TICK_RECOVER_THESHOLD (20) //unit is ms

/* 1 set of pwm_in init trigger + 1bit start_bit + 64bit data + 1bit end_bit
 * = 67 * 2(each bit include rising & falling cnt) = 134 */
#define IR_PWM_IN_LEN (134)
#define START_BIT_IDX (3)
#define STOP_BIT_IDX (133)
#define DATA_START_IDX (4)
/* use ping-pong buffer with only double size */
#define BUF_SIZE (IR_PWM_IN_LEN<<1)


#ifdef DBG_INFO_DISPLAY
extern void UART_PRINT(const char * format, ... );
#define DBG_INFO(str, args...) UART_PRINT(str, ##args)
#else
#define DBG_INFO(str, args...)
#endif

#define DMA_ADDR_DEF(IDX) (DMA##IDX)
#define DMA_ADDR(IDX) DMA_ADDR_DEF(IDX)
#define DMA_CH_ADDR_DEF(IDX, CH) (DMA##IDX##_Channel##CH)
#define DMA_CH_ADDR(IDX, CH) DMA_CH_ADDR_DEF(IDX, CH)

#if (IR_DMA_CHANNEL==1)
#define DMA_CH_IRQN_DEF(IDX) (DMA##IDX##_Channel1_IRQn)
#elif (IR_DMA_CHANNEL==2 || IR_DMA_CHANNEL==3)
#define DMA_CH_IRQN_DEF(IDX) (DMA##IDX##_Channel2_3_IRQn)
#elif (IR_DMA_CHANNEL==4 || IR_DMA_CHANNEL==5)
#define DMA_CH_IRQN_DEF(IDX) (DMA##IDX##_Channel4_5_IRQn)
#endif

#define DMA_CH_IRQN(IDX) DMA_CH_IRQN_DEF(IDX)

#define IR_DMA DMA_ADDR(IR_DMA_IDX)
#define IR_DMA_CH DMA_CH_ADDR(IR_DMA_IDX, IR_DMA_CHANNEL)
#define IR_DMA_IRQN DMA_CH_IRQN(IR_DMA_IDX)

#define REG_IR_DMA_ISR (IR_DMA->ISR)
#define REG_IR_DMA_IFCR (IR_DMA->IFCR)
#define REG_IR_DMA_CPAR (IR_DMA_CH->CPAR)
#define REG_IR_DMA_CCR (IR_DMA_CH->CCR)
#define REG_IR_DMA_CNDTR (IR_DMA_CH->CNDTR)
#define REG_IR_DMA_CMAR (IR_DMA_CH->CMAR)

#define DMA_ISR_TCIF(CH) (DMA_ISR_TCIF##CH)
#define DMA_ISR_TEIF(CH) (DMA_ISR_TEIF##CH)
#define DMA_IFCR_CGIF(CH) (DMA_IFCR_CGIF##CH)
#define DMA_IFCR_CTCIF(CH) (DMA_IFCR_CTCIF##CH)

#define DMA_ISR_TCIF_CH(CH) (DMA_ISR_TCIF(CH))
#define DMA_ISR_TEIF_CH(CH) (DMA_ISR_TEIF(CH))
#define DMA_IFCR_CGIF_CH(CH) (DMA_IFCR_CGIF(CH))
#define DMA_IFCR_CTCIF_CH(CH) (DMA_IFCR_CTCIF(CH))

#define IR_DMA_ISR_TCIF (DMA_ISR_TCIF_CH(IR_DMA_CHANNEL))
#define IR_DMA_ISR_TEIF (DMA_ISR_TEIF_CH(IR_DMA_CHANNEL))
#define IR_DMA_IFCR_CGIF (DMA_IFCR_CGIF_CH(IR_DMA_CHANNEL))
#define IR_DMA_IFCR_CTCIF (DMA_IFCR_CTCIF_CH(IR_DMA_CHANNEL))

#define REG_IR_CR1 (IR_TIM->CR1)
#define REG_IR_DIER (IR_TIM->DIER)
#define REG_IR_CCER (IR_TIM->CCER)
#define REG_IR_DCR (IR_TIM->DCR)
#define REG_IR_DMAR (IR_TIM->DMAR)

#define BIT0 (0x01)
#define BIT1 (0x02)
#define BIT2 (0x04)
#define BIT3 (0x08)
#define BIT4 (0x10)
#define BIT5 (0x20)
#define BIT6 (0x40)
#define BIT7 (0x80)

typedef struct _ST_IR_DATA_BYTE0 {
    unsigned temp_tens:3 ;
    unsigned body_sens:1 ;
    unsigned temp_ones:4 ;
} ST_IR_DATA_BYTE0;

typedef struct _ST_IR_DATA_BYTE1 {
    unsigned curtime_hour_tens:1 ;
    unsigned curtime_AMPM:1 ;
    unsigned power_en:1 ;
    unsigned sleep_en:1 ;
    unsigned curtime_hour_ones:4 ;
} ST_IR_DATA_BYTE1;

typedef struct _ST_IR_DATA_BYTE2 {
    unsigned curtime_min_tens:3 ;
    unsigned ABCode:1 ;
    unsigned curtime_min_ones:4 ;
} ST_IR_DATA_BYTE2;

typedef struct _ST_IR_DATA_BYTE3 {
    unsigned ontime_hour_tens:1 ;
    unsigned ontime_AMPM:1 ;
    unsigned ontime_en:1 ;
    unsigned warm_en:1 ;
    unsigned ontime_hour_ones:4 ;
} ST_IR_DATA_BYTE3;

typedef struct _ST_IR_DATA_BYTE4 {
    unsigned ontime_min_tens:4 ;
    unsigned fan_en:1 ;
    unsigned dry_en:1 ;
    unsigned cool_en:1 ;
    unsigned auto_en:1 ;
} ST_IR_DATA_BYTE4;

typedef struct _ST_IR_DATA_BYTE5 {
    unsigned offtime_hour_tens:1 ;
    unsigned offtime_AMPM:1 ;
    unsigned offtime_en:1 ;
    unsigned airclean_en:1 ;
    unsigned offtime_hour_ones:4 ;
} ST_IR_DATA_BYTE5;

typedef struct _ST_IR_DATA_BYTE6 {
    unsigned offtime_min_tens:4 ;
    unsigned dir_manual_en:1 ;
    unsigned dir_auto_en:1 ;
    unsigned auto_temp_down:1 ;
    unsigned auto_temp_up:1 ;
} ST_IR_DATA_BYTE6;

typedef struct _ST_IR_DATA_BYTE7 {
    unsigned fan_high_en:1 ;
    unsigned fan_mid_en:1 ;
    unsigned fan_low_en:1 ;
    unsigned fan_auto_en:1 ;
    unsigned checksum:4 ;
} ST_IR_DATA_BYTE7;

typedef struct _ST_4BIT_DATA {
    unsigned low_4bit:4 ;
    unsigned high_4bit:4 ;
} ST_4BIT_DATA;

#define UN_IR_DATA_DECLARE(IDX) \
    typedef union _UN_IR_DATA_BYTE##IDX {\
        ST_IR_DATA_BYTE##IDX stByte;\
        ST_4BIT_DATA st4Bits;\
        uint8_t u8Byte;\
    } UN_IR_DATA_BYTE##IDX

UN_IR_DATA_DECLARE(0);
UN_IR_DATA_DECLARE(1);
UN_IR_DATA_DECLARE(2);
UN_IR_DATA_DECLARE(3);
UN_IR_DATA_DECLARE(4);
UN_IR_DATA_DECLARE(5);
UN_IR_DATA_DECLARE(6);
UN_IR_DATA_DECLARE(7);

typedef struct _ST_IR_UN_DATA {
    UN_IR_DATA_BYTE0 unByte0;
    UN_IR_DATA_BYTE1 unByte1;
    UN_IR_DATA_BYTE2 unByte2;
    UN_IR_DATA_BYTE3 unByte3;
    UN_IR_DATA_BYTE4 unByte4;
    UN_IR_DATA_BYTE5 unByte5;
    UN_IR_DATA_BYTE6 unByte6;
    UN_IR_DATA_BYTE7 unByte7;
} ST_IR_UN_DATA;


extern TIM_HandleTypeDef IR_TIM_HANDLE;
extern DMA_HandleTypeDef IR_TIM_DMA_HANDLE;

static uint16_t IR_BUF[BUF_SIZE];
static ST_IR_UN_DATA gstUnData;
static ST_IR_DATA gstData;

#ifdef TICK_RECOVER_CHECK
static uint32_t gu32CNDTR_Cur, gu32CNDTR_Pre;
static uint32_t gu32Tick_Pre;
#endif


#if 0 /* disable since IRQ is not used now */
static HAL_StatusTypeDef IR_Private_IRQHandler(void)
{
    /* Transfer Complete Interrupt management ***********************************/
    if (REG_IR_DMA_ISR & IR_DMA_ISR_TCIF) {

        /* Clear the transfer complete flag */
        REG_IR_DMA_IFCR = IR_DMA_IFCR_CTCIF;

        /* Transfer complete callback */
        /* hdma->XferCpltCallback(hdma); */

    }
    /* Transfer Error Interrupt management ***************************************/
    else if (REG_IR_DMA_ISR & IR_DMA_ISR_TEIF) {

        /* When a DMA transfer error occurs */
        /* A hardware clear of its EN bits is performed */
        /* Then, disable all DMA interrupts */
        REG_IR_DMA_CCR &= ~(DMA_IT_TC | DMA_IT_TE);

        /* Clear all flags */
        REG_IR_DMA_IFCR = IR_DMA_IFCR_CGIF;

        /* Update error code */
        (&IR_TIM_DMA_HANDLE)->ErrorCode = HAL_DMA_ERROR_TE;

        /* Change the DMA state */
        (&IR_TIM_DMA_HANDLE)->State = HAL_DMA_STATE_READY;

        /* Transfer error callback */
        /* hdma->XferErrorCallback(hdma); */

    }

    return HAL_OK;
}

/**
  Disable Code Generation in CubeMX NVIC Configuration for DMA IRQ handler.
  Replace by own IRQ handler function instead.
  */
void DMA1_Channel4_5_IRQHandler(void)
{
    IR_Private_IRQHandler();
}

HAL_StatusTypeDef Drv_IR_CB_Register(IR_CBFunc cb)
{
    return HAL_OK;
}
#endif

static uint8_t IR_ByteDecode(uint16_t *pBuf)
{
    uint8_t byte = 0;

    if (pBuf[1] > DECODE_TH_HIGHBIT) { //bit0
        byte = BIT0;
    }

    if (pBuf[3] > DECODE_TH_HIGHBIT) { //bit1
        byte |= BIT1;
    }

    if (pBuf[5] > DECODE_TH_HIGHBIT) { //bit2
        byte |= BIT2;
    }

    if (pBuf[7] > DECODE_TH_HIGHBIT) { //bit3
        byte |= BIT3;
    }

    if (pBuf[9] > DECODE_TH_HIGHBIT) { //bit4
        byte |= BIT4;
    }

    if (pBuf[11] > DECODE_TH_HIGHBIT) { //bit5
        byte |= BIT5;
    }

    if (pBuf[13] > DECODE_TH_HIGHBIT) { //bit6
        byte |= BIT6;
    }

    if (pBuf[15] > DECODE_TH_HIGHBIT) { //bit7
        byte |= BIT7;
    }

    return byte;
}

static HAL_StatusTypeDef IR_Decode(uint16_t *pBuf)
{
    uint8_t checksum;

    /* do byte decode first */
    gstUnData.unByte0.u8Byte = IR_ByteDecode(&pBuf[0]);
    gstUnData.unByte1.u8Byte = IR_ByteDecode(&pBuf[16]);
    gstUnData.unByte2.u8Byte = IR_ByteDecode(&pBuf[32]);
    gstUnData.unByte3.u8Byte = IR_ByteDecode(&pBuf[48]);
    gstUnData.unByte4.u8Byte = IR_ByteDecode(&pBuf[64]);
    gstUnData.unByte5.u8Byte = IR_ByteDecode(&pBuf[80]);
    gstUnData.unByte6.u8Byte = IR_ByteDecode(&pBuf[96]);
    gstUnData.unByte7.u8Byte = IR_ByteDecode(&pBuf[112]);

    /* check checksum */
    checksum = gstUnData.unByte0.st4Bits.low_4bit + gstUnData.unByte0.st4Bits.high_4bit
        + gstUnData.unByte1.st4Bits.low_4bit + gstUnData.unByte1.st4Bits.high_4bit
        + gstUnData.unByte2.st4Bits.low_4bit + gstUnData.unByte2.st4Bits.high_4bit
        + gstUnData.unByte3.st4Bits.low_4bit + gstUnData.unByte3.st4Bits.high_4bit
        + gstUnData.unByte4.st4Bits.low_4bit + gstUnData.unByte4.st4Bits.high_4bit
        + gstUnData.unByte5.st4Bits.low_4bit + gstUnData.unByte5.st4Bits.high_4bit
        + gstUnData.unByte6.st4Bits.low_4bit + gstUnData.unByte6.st4Bits.high_4bit
        + gstUnData.unByte7.st4Bits.low_4bit;

    /* take the least 4bit */
    checksum &= 0x0F;

    DBG_INFO("cal_checksum = 0x%x, dec_checksum = 0x%x\r\n", checksum, gstUnData.unByte7.stByte.checksum);
    if (checksum != gstUnData.unByte7.stByte.checksum) {
        return HAL_ERROR;
    }

    /* change to more readable data structure */
    gstData.set_temp = gstUnData.unByte0.stByte.temp_tens*10 + gstUnData.unByte0.stByte.temp_ones;
    gstData.body_sens = gstUnData.unByte0.stByte.body_sens;
    gstData.curtime_hour = gstUnData.unByte1.stByte.curtime_hour_tens*10 + gstUnData.unByte1.stByte.curtime_hour_ones;
    gstData.curtime_min = gstUnData.unByte2.stByte.curtime_min_tens*10 + gstUnData.unByte2.stByte.curtime_min_ones;
    gstData.curtime_AMPM = gstUnData.unByte1.stByte.curtime_AMPM;
    gstData.power_en = gstUnData.unByte1.stByte.power_en;
    gstData.sleep_en = gstUnData.unByte1.stByte.sleep_en;
    gstData.ABCode = gstUnData.unByte2.stByte.ABCode;
    gstData.ontime_hour = gstUnData.unByte3.stByte.ontime_hour_tens*10 + gstUnData.unByte3.stByte.ontime_hour_ones;
    gstData.ontime_min = gstUnData.unByte4.stByte.ontime_min_tens*10;
    gstData.ontime_AMPM = gstUnData.unByte3.stByte.ontime_AMPM;
    gstData.ontime_en = gstUnData.unByte3.stByte.ontime_en;

    /* hour off mode */
    if (0x0F == gstUnData.unByte6.stByte.offtime_min_tens) {
        gstData.hour_off_en = 1;
        gstData.hour_off_hour = gstUnData.unByte5.stByte.offtime_hour_ones;
        gstData.offtime_hour = 0;
        gstData.offtime_min = 0;
    } else {
        gstData.hour_off_en = 0;
        gstData.hour_off_hour = 0;
        gstData.offtime_hour = gstUnData.unByte5.stByte.offtime_hour_tens*10 + gstUnData.unByte5.stByte.offtime_hour_ones;
        gstData.offtime_min = gstUnData.unByte6.stByte.offtime_min_tens*10;
    }

    /* offtime and hour-off is exclusive or not? needs to check... */
    gstData.offtime_AMPM = gstUnData.unByte5.stByte.offtime_AMPM;
    gstData.offtime_en = gstUnData.unByte5.stByte.offtime_en;

    gstData.warm_en = gstUnData.unByte3.stByte.warm_en;
    gstData.cool_en = gstUnData.unByte4.stByte.cool_en;
    gstData.dry_en = gstUnData.unByte4.stByte.dry_en;
    gstData.fan_en = gstUnData.unByte4.stByte.fan_en;
    gstData.auto_en = gstUnData.unByte4.stByte.auto_en;
    gstData.airclean_en = gstUnData.unByte5.stByte.airclean_en;
    gstData.dir_manual_en = gstUnData.unByte6.stByte.dir_manual_en;
    gstData.dir_auto_en = gstUnData.unByte6.stByte.dir_auto_en;
    gstData.auto_temp_down = gstUnData.unByte6.stByte.auto_temp_down;
    gstData.auto_temp_up = gstUnData.unByte6.stByte.auto_temp_up;
    gstData.fan_high_en = gstUnData.unByte7.stByte.fan_high_en;
    gstData.fan_mid_en = gstUnData.unByte7.stByte.fan_mid_en;
    gstData.fan_low_en = gstUnData.unByte7.stByte.fan_low_en;
    gstData.fan_auto_en = gstUnData.unByte7.stByte.fan_auto_en;

    return HAL_OK;
}

void Drv_IR_Init(void)
{
    /* Disable the IRQ which is enable by CubeMX in default */
    HAL_NVIC_DisableIRQ(IR_DMA_IRQN);

    /* DMA pre-set part */
    /* pre-set DMA CPAR */
    REG_IR_DMA_CPAR = (uint32_t)&(REG_IR_DMAR);

    /* pre-set TIM DMA setting */
    REG_IR_DCR = IR_TIM_DMABASE_CCR | TIM_DMABURSTLENGTH_2TRANSFERS;

    /* Enable the TIM DMA Request */
    REG_IR_DIER |= IR_TIM_DMASRC;

    /* set buffer content with init value */
    /* memset((void*)IR_BUF, 0xFF, sizeof(uint16_t)*BUF_SIZE); */
    /* set the head-byte is enough */
    IR_BUF[0] = 0xFFFF;
    IR_BUF[ IR_PWM_IN_LEN ] = 0xFFFF;
#ifndef TICK_RECOVER_CHECK
    IR_BUF[ START_BIT_IDX ] = 0xFFFF;
    IR_BUF[ IR_PWM_IN_LEN + START_BIT_IDX ] = 0xFFFF;
#endif

    /* re-write */
    /* HAL_TIM_IC_Start(&IR_TIM_HANDLE, IR_TIM_PWM_IN_FALLING_CH); */
    /* HAL_TIM_IC_Start(&IR_TIM_HANDLE, IR_TIM_PWM_IN_RISING_CH); */

    /* enable input capture on bothe rising and falling edge */
    REG_IR_CCER |= (TIM_CCx_ENABLE << IR_TIM_PWM_IN_FALLING_CH) | (TIM_CCx_ENABLE << IR_TIM_PWM_IN_RISING_CH);

    /* set counter enable */
    REG_IR_CR1 |= TIM_CR1_CEN;


}

void Drv_IR_Receive_DMA_Start(void)
{
    /* re-write */
    /* if (HAL_OK != HAL_TIM_DMABurst_MultiReadStart(&IR_TIM_HANDLE, IR_TIM_DMABASE_CCR, IR_TIM_DMASRC, (uint32_t*)IR_BUF, TIM_DMABURSTLENGTH_2TRANSFERS, BUF_SIZE)) { */
    /* return HAL_ERROR; */
    /* } */

    /* some settings pre-set in init() */

    /* set CNDTR */
    REG_IR_DMA_CNDTR = BUF_SIZE;

    /* set CMAR */
    REG_IR_DMA_CMAR = (uint32_t)(&IR_BUF);

    /* enable DMA channel */
    REG_IR_DMA_CCR |= DMA_CCR_EN;

}

void Drv_IR_Receive_DMA_Stop(void)
{
    /* re-write */
    /* if (HAL_OK != HAL_TIM_DMABurst_ReadStop(&IR_TIM_HANDLE, TIM_DMA_CC1)) { */
    /* return HAL_ERROR; */
    /* } */

    /* disable DMA channel */
    REG_IR_DMA_CCR &= ~DMA_CCR_EN;

}

HAL_StatusTypeDef Drv_IR_GetData(ST_IR_DATA **ppstData)
{
#ifdef TICK_RECOVER_CHECK
    gu32CNDTR_Cur = REG_IR_DMA_CNDTR;

    if (gu32CNDTR_Cur <= IR_PWM_IN_LEN) {
        /* means the first-half of the ping-pong buffer is filled */

        /* also means the second-half of the ping-pong buffer is filling now */

        /* tick recover check */
        /* if the ping-pong buffer is not filled complete for too long, it will be detected as error */
        if (gu32CNDTR_Cur != IR_PWM_IN_LEN) {
            if (gu32CNDTR_Pre == gu32CNDTR_Cur) {
                if (HAL_GetTick() - gu32Tick_Pre > TICK_RECOVER_THESHOLD) {
                    return HAL_ERROR;
                }
            } else {
                gu32CNDTR_Pre = gu32CNDTR_Cur;
                gu32Tick_Pre = HAL_GetTick();
            }
        }

        /* check the content is update after init or not */
        if (0xFFFF == IR_BUF[0]) {
            return HAL_BUSY;
        }

        /* check data correctness */
        /* check start_bit and stop_bit first */
        if (IR_BUF[ START_BIT_IDX ] < DECODE_TH_STARTBIT || IR_BUF[ STOP_BIT_IDX ] < DECODE_TH_STOPBIT) {
            DBG_INFO("Error @ %s, line = %d\r\n", __FUNCTION__, __LINE__);
            return HAL_ERROR;
        }

        /* set to init value after check */
        IR_BUF[0] = 0xFFFF;

        /* do IR decode */
        if (HAL_ERROR == IR_Decode(&IR_BUF[ DATA_START_IDX ])) {
            DBG_INFO("Error @ %s, line = %d\r\n", __FUNCTION__, __LINE__);
            return HAL_ERROR;
        }

    } else {
        /* means the second-half of the ping-pong buffer is filled */

        /* also means the first-half of the ping-pong buffer is filling now */

        /* tick recover check */
        /* if the ping-pong buffer is not filled complete for too long, it will be detected as error */
        if (gu32CNDTR_Cur != BUF_SIZE) {
            if (gu32CNDTR_Pre == gu32CNDTR_Cur) {
                if (HAL_GetTick() - gu32Tick_Pre > TICK_RECOVER_THESHOLD) {
                    DBG_INFO("Error @ %s, line = %d\r\n", __FUNCTION__, __LINE__);
                    return HAL_ERROR;
                }
            } else {
                gu32CNDTR_Pre = gu32CNDTR_Cur;
                gu32Tick_Pre = HAL_GetTick();
            }
        }

        /* check the content is update after init or not */
        if (0xFFFF == IR_BUF[ IR_PWM_IN_LEN ]) {
            return HAL_BUSY;
        }

        /* check data correctness */
        /* check start_bit and stop_bit first */
        if (IR_BUF[ IR_PWM_IN_LEN + START_BIT_IDX ] < DECODE_TH_STARTBIT || IR_BUF[ IR_PWM_IN_LEN + STOP_BIT_IDX ] < DECODE_TH_STOPBIT) {
            DBG_INFO("Error @ %s, line = %d\r\n", __FUNCTION__, __LINE__);
            return HAL_ERROR;
        }

        /* set to init value after check */
        IR_BUF[ IR_PWM_IN_LEN ] = 0xFFFF;

        /* do IR decode */
        if (HAL_ERROR == IR_Decode(&IR_BUF[ IR_PWM_IN_LEN + DATA_START_IDX ])) {
            DBG_INFO("Error @ %s, line = %d\r\n", __FUNCTION__, __LINE__);
            return HAL_ERROR;
        }

    }

#else

    if (REG_IR_DMA_CNDTR <= IR_PWM_IN_LEN) {
        /* means the first-half of the ping-pong buffer is filled */

        /* also means the second-half of the ping-pong buffer is filling now */

        /* error check for error recover */
        /* check the start bit of current filling one */
        if (IR_BUF[ IR_PWM_IN_LEN + START_BIT_IDX ] < DECODE_TH_STARTBIT) {
            /* the START_BIT should be correct */
            return HAL_ERROR;
        }

        /* check the content is update after init or not */
        if (0xFFFF == IR_BUF[0]) {
            return HAL_BUSY;
        }

        /* check data correctness */
        /* check start_bit and stop_bit first */
        if (IR_BUF[ START_BIT_IDX ] < DECODE_TH_STARTBIT || IR_BUF[ STOP_BIT_IDX ] < DECODE_TH_STOPBIT) {
            return HAL_ERROR;
        }

        /* set to init value after check */
        IR_BUF[0] = 0xFFFF;
        IR_BUF[ START_BIT_IDX ] = 0xFFFF;

        /* do IR decode */
        if (HAL_ERROR == IR_Decode(&IR_BUF[ DATA_START_IDX ])) {
            return HAL_ERROR;
        }

    } else {
        /* means the second-half of the ping-pong buffer is filled */

        /* also means the first-half of the ping-pong buffer is filling now */

        /* error check for error recover */
        /* check the start bit of current filling one */
        if (IR_BUF[ START_BIT_IDX ] < DECODE_TH_STARTBIT) {
            /* the START_BIT should be correct */
            return HAL_ERROR;
        }

        /* check the content is update after init or not */
        if (0xFFFF == IR_BUF[ IR_PWM_IN_LEN ]) {
            return HAL_BUSY;
        }

        /* check data correctness */
        /* check start_bit and stop_bit first */
        if (IR_BUF[ IR_PWM_IN_LEN + START_BIT_IDX ] < DECODE_TH_STARTBIT || IR_BUF[ IR_PWM_IN_LEN + STOP_BIT_IDX ] < DECODE_TH_STOPBIT) {
            return HAL_ERROR;
        }

        /* set to init value after check */
        IR_BUF[ IR_PWM_IN_LEN ] = 0xFFFF;
        IR_BUF[ IR_PWM_IN_LEN + START_BIT_IDX ] = 0xFFFF;

        /* do IR decode */
        if (HAL_ERROR == IR_Decode(&IR_BUF[ IR_PWM_IN_LEN + DATA_START_IDX ])) {
            return HAL_ERROR;
        }

    }

#endif

    /* return the address of the stored decode data */
    *ppstData = &gstData;

    return HAL_OK;
}

void Drv_IR_Error_Recover(void)
{
    /* disable DMA channel */
    REG_IR_DMA_CCR &= ~DMA_CCR_EN;

    /* set CNDTR */
    REG_IR_DMA_CNDTR = BUF_SIZE;

    /* set CMAR */
    REG_IR_DMA_CMAR = (uint32_t)(&IR_BUF);

    /* reset the head bytes in the ping-pong buffer to init value */
    IR_BUF[0] = 0xFFFF;
    IR_BUF[ IR_PWM_IN_LEN ] = 0xFFFF;
#ifndef TICK_RECOVER_CHECK
    IR_BUF[ START_BIT_IDX ] = 0xFFFF;
    IR_BUF[ IR_PWM_IN_LEN + START_BIT_IDX ] = 0xFFFF;
#endif

    /* enable DMA channel */
    REG_IR_DMA_CCR |= DMA_CCR_EN;

}
