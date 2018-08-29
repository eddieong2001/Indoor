/**********************************************************************
*                       For step motor control                       *
**********************************************************************/

#ifndef __STEP_CTRL_H__
#define __STEP_CTRL_H__

#include "stm32f0xx_hal.h"

void step_cw(uint16_t steps);
void step_ccw(uint16_t steps);

#endif //__STEP_CTRL_H__
