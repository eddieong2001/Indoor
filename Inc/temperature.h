/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H
/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_hal.h"

#define TEMP_ADC (ADC1)
#define TEMP_ADC_HANDLE (hadc)

#define TEMP_CP_IN_ADC_CHANNEL (ADC_CHANNEL_15)
#define TEMP_CP_OUT_ADC_CHANNEL (ADC_CHANNEL_8)
#define TEMP_ENV_ADC_CHANNEL (ADC_CHANNEL_9)

typedef enum {
    EN_TEMP_CP_IN = 0,
    EN_TEMP_CP_OUT,
    EN_TEMP_ENV
} EN_TEMP;

/*
 * Should call this API first before use other Temp API.
 *
 * return HAL_OK: Temp ADC enable success
 * return HAL_ERROR: Temp ADC enable fail
 */
HAL_StatusTypeDef Drv_Temp_ADC_Enable(void);

/*
 * Call this API to start ADC sampling to the indicate temperature sensor channel.
 * If you want to change the indicated channel, should called SampleStop first,
 * and then call SampleStart to set a new channel index.
 *
 * return HAL_OK: Sample start success
 * return HAL_BUSY: Previous SampleStop not finished yet
 * return HAL_ERROR: ADC sample is already started (Not call SampleStop yet)
 */
HAL_StatusTypeDef Drv_Temp_SampleStart(EN_TEMP enTemp);

/*
 * Call this API to stop ADC sampling of the current temperature sensor channel
 * which is indicated by last SampleStart API.
 *
 * return HAL_OR: Sample stop success
 * return HAL_ERROR: Sample start is not executed yet
 */
HAL_StatusTypeDef Drv_Temp_SampleStop(void);

/*
 * Call this API to get the temperature value of the current temperature sensor channel
 * which is indicated by last SampleStart API.
 * Can calling this API to get the latest temperature of current channel without calling SampleStart again.
 *
 * return HAL_OK: Get value OK
 * return HAL_ERROR: The first sample value after SampleStart is not ready yet
 */
HAL_StatusTypeDef Drv_Temp_GetTemp(int16_t *pTempVal);

#endif  /* __TEMPERATURE_H */
