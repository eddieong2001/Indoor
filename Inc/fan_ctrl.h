/**********************************************************************
*             Porting from Haoren's outdoor source code              *
**********************************************************************/

#ifndef __FAN_CTRL_H__
#define __FAN_CTRL_H__

// #include "inverter_misc.h"
#include "stm32f0xx_hal.h"

HAL_StatusTypeDef FanInit(uint16_t fanInitLevel);
void FanStart(void);
void FanStop(void);
uint16_t FanGetSpeed(void);
void FanSetLevel(uint16_t targetLevel);
uint16_t FanGetLevel(void);

#endif /* __FAN_CTRL_H__ */
