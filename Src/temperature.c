/**
 ******************************************************************************
 * File Name          : temperature.c
 * Description        : This file provide the customize config to get temperature value
 ******************************************************************************
 *  Since the DMA resource are limit, implement in no DMA way.
 *
 *  Set ADC with only one channel to one temperature channel a time.
 *  Set to continues conversion mode, and set overflow mode to overwrite.
 *
 *  No use interrupt, read ADC data register directly.
 *  In order to reduce the system overhead
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "temperature.h"
#include "temperature_tbl.h"

#define REG_ISR (TEMP_ADC->ISR)
#define REG_CR (TEMP_ADC->CR)
#define REG_CHSELR (TEMP_ADC->CHSELR)
#define REG_DR (TEMP_ADC->DR)

#define TEMP_CP_IN_CHSELR_SETTING (ADC_CHSELR_CHANNEL(TEMP_CP_IN_ADC_CHANNEL))
#define TEMP_CP_OUT_CHSELR_SETTING (ADC_CHSELR_CHANNEL(TEMP_CP_OUT_ADC_CHANNEL))
#define TEMP_ENV_CHSELR_SETTING (ADC_CHSELR_CHANNEL(TEMP_ENV_ADC_CHANNEL))

extern ADC_HandleTypeDef TEMP_ADC_HANDLE;

/* local function declaration */
#define FUNC_GET_TEMP_FROM_TBL(TBL)\
    static int16_t getTemp##TBL(uint16_t adcVal)\
    {\
        uint16_t start = 0;\
        uint16_t end = TBL##_SIZE - 1;\
        uint16_t mid;\
        \
        if (adcVal <= u16_##TBL[0]) {\
            return TBL##_TEMP_START;\
        } else if (adcVal >= u16_##TBL[TBL##_SIZE - 1]) {\
            return ((int16_t)TBL##_SIZE - 1 + TBL##_TEMP_START);\
        }\
        \
        while (start < end) {\
            mid = (start + end) >> 1;\
            \
            if (adcVal >= u16_##TBL[mid]) {\
                if (adcVal <= u16_##TBL[mid+1]) {\
                    return (mid + TBL##_TEMP_START);\
                }\
                start = mid;\
            } else {\
                end = mid;\
            }\
        }\
        \
        return (-9999); /* this should never happened */\
    }\

FUNC_GET_TEMP_FROM_TBL(TBL103F397F)
/* FUNC_GET_TEMP_FROM_TBL(TBL473F397F) */

static uint32_t u32CHSELR_Array[] = {
    TEMP_CP_IN_CHSELR_SETTING,
    TEMP_CP_OUT_CHSELR_SETTING,
    TEMP_ENV_CHSELR_SETTING,
};

HAL_StatusTypeDef Drv_Temp_ADC_Enable(void)
{
    /* do ADC calibration in single-ended mode */
    if (HAL_OK != HAL_ADCEx_Calibration_Start(&TEMP_ADC_HANDLE)) {
        return HAL_ERROR;
    }

    /* uses HAL_ADC_Start to enable ADC */
    if (HAL_OK != HAL_ADC_Start(&TEMP_ADC_HANDLE)) {
        return HAL_ERROR;
    }

    /* and stop ADC at beginning */
    REG_CR |= ADC_CR_ADSTP;

    return HAL_OK;
}

HAL_StatusTypeDef Drv_Temp_SampleStart(EN_TEMP enTemp)
{
    /* ADSTP = 1 means stop regular conversions ongoing */
    if (ADC_CR_ADSTP & REG_CR) {
        return HAL_BUSY;
    }

    /* ADSTART = 1 means the regular conversion is already start */
    if (ADC_CR_ADSTART & REG_CR) {
        return HAL_ERROR;
    }

    /* set select channel to SRQ */
    REG_CHSELR = u32CHSELR_Array[enTemp];

    /* enable ADC start bit */
    REG_CR |= ADC_CR_ADSTART;

    return HAL_OK;
}

HAL_StatusTypeDef Drv_Temp_SampleStop(void)
{
    /* Stop can only be set when ADSTART=1 */
    if (!(ADC_CR_ADSTART & REG_CR)) {
        return HAL_ERROR;
    }

    REG_CR |= ADC_CR_ADSTP;

    return HAL_OK;
}

HAL_StatusTypeDef Drv_Temp_GetTemp(int16_t *pTempVal)
{
    /* EOC flag should be raised after first ADC sample conversion complete */
    if (!(ADC_ISR_EOC & REG_ISR)) {
        return HAL_ERROR;
    }

    *pTempVal = getTempTBL103F397F((uint16_t)REG_DR);

    return HAL_OK;
}
