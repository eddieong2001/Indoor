/**********************************************************************
 *                       For step motor control                       *
 **********************************************************************/

#include "step_ctrl.h"

#define STEP_DELAY (10)

#define PIN_HIGH(X)\
    HAL_GPIO_WritePin(GPIO_OUT_LN2003_IN##X##_GPIO_Port, GPIO_OUT_LN2003_IN##X##_Pin, GPIO_PIN_SET)

#define PIN_LOW(X)\
    HAL_GPIO_WritePin(GPIO_OUT_LN2003_IN##X##_GPIO_Port, GPIO_OUT_LN2003_IN##X##_Pin, GPIO_PIN_RESET)

void step_cw(uint16_t steps)
{
    uint16_t i;

    if (steps > 0) {

        PIN_HIGH(0);
        for (i = 0; i < steps; ++i) {
            PIN_LOW(3);
            HAL_Delay(STEP_DELAY);

            PIN_HIGH(1);
            HAL_Delay(STEP_DELAY);

            PIN_LOW(0);
            HAL_Delay(STEP_DELAY);

            PIN_HIGH(2);
            HAL_Delay(STEP_DELAY);

            PIN_LOW(1);
            HAL_Delay(STEP_DELAY);

            PIN_HIGH(3);
            HAL_Delay(STEP_DELAY);

            PIN_LOW(2);
            HAL_Delay(STEP_DELAY);

            PIN_HIGH(0);
            HAL_Delay(STEP_DELAY);
        }

        //reset all pins in the end
        PIN_LOW(0);
        PIN_LOW(1);
        PIN_LOW(2);
        PIN_LOW(3);
    }
}

void step_ccw(uint16_t steps)
{
    uint16_t i;

    if (steps > 0) {

        PIN_HIGH(3);
        for (i = 0; i < steps; ++i) {
            PIN_LOW(0);
            HAL_Delay(STEP_DELAY);

            PIN_HIGH(2);
            HAL_Delay(STEP_DELAY);

            PIN_LOW(3);
            HAL_Delay(STEP_DELAY);

            PIN_HIGH(1);
            HAL_Delay(STEP_DELAY);

            PIN_LOW(2);
            HAL_Delay(STEP_DELAY);

            PIN_HIGH(0);
            HAL_Delay(STEP_DELAY);

            PIN_LOW(1);
            HAL_Delay(STEP_DELAY);

            PIN_HIGH(3);
            HAL_Delay(STEP_DELAY);
        }

        //reset all pins in the end
        PIN_LOW(0);
        PIN_LOW(1);
        PIN_LOW(2);
        PIN_LOW(3);
    }
}
