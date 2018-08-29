/**********************************************************************
*             Porting from Haoren's outdoor source code              *
**********************************************************************/

/* #include "inverter_config.h" */
/* #include "inverter_misc.h" */
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_tim.h"

#define DBG_INFO_DISPLAY

#ifdef DBG_INFO_DISPLAY
extern void UART_PRINT(const char * format, ... );
#define DBG_INFO(str, args...) UART_PRINT(str, ##args)
#else
#define DBG_INFO(str, args...)
#endif

#define TIM14_PRESCALER (11) /* it means prescaler = 12 */
#define FAN_PWM_FREQ (2000) /*40000*/

/* void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim); */

uint16_t fanLevel; // 0(slowest) to 100(fastest)
uint16_t fanPwmArr;

HAL_StatusTypeDef _FanConfig(void)
{
    /* Re-config timer settins */
    LL_TIM_SetPrescaler(TIM14, TIM14_PRESCALER);
    LL_TIM_SetAutoReload(TIM14, fanPwmArr);

    return HAL_OK;
}

HAL_StatusTypeDef FanInit(uint16_t fanInitLevel)
{
    uint32_t apb1TimerMultiplier;
    uint32_t tim14Freq;

    apb1TimerMultiplier = (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE_1) >> RCC_CFGR_PPRE_Pos] == 0) ? 1 : 2;
    tim14Freq = HAL_RCC_GetPCLK1Freq() * apb1TimerMultiplier;

    fanPwmArr = (tim14Freq / (TIM14_PRESCALER + 1)) / FAN_PWM_FREQ;
    fanLevel = fanInitLevel;

    DBG_INFO("FAN PWM settins:\r\n");
    DBG_INFO("PWM prescaler = %d\r\n", TIM14_PRESCALER);
    DBG_INFO("PWM frequency = %d\r\n", FAN_PWM_FREQ);
    DBG_INFO("PWM ARR = %d\r\n", fanPwmArr);
    
    DBG_INFO("apb1TimerMultiplier = %d\r\n", apb1TimerMultiplier);
    DBG_INFO("tim14Freq = %d\r\n", tim14Freq);
    DBG_INFO("fanInitLevel = %d\r\n", fanInitLevel);

    _FanConfig();

    return HAL_OK;
}

uint16_t FanGetSpeed(void)
{
    uint32_t ccr1;

    if (LL_TIM_IsActiveFlag_CC1(TIM3)) {
        ccr1 = LL_TIM_IC_GetCaptureCH1(TIM3);
        LL_TIM_ClearFlag_CC1(TIM3);
        return (uint16_t)((5 * HAL_RCC_GetHCLKFreq() / (LL_TIM_GetPrescaler(TIM3) + 1)) / ccr1); // 5: 60/12(squar in a circle)
    } else {
        return 0;
    }
}

void FanStart(void)
{
    /* Enable channel */
    LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);

    /* Enable Counter */
    LL_TIM_EnableCounter(TIM14);
    LL_TIM_EnableCounter(TIM3);

    LL_TIM_GenerateEvent_UPDATE(TIM14);
    LL_TIM_GenerateEvent_UPDATE(TIM3);

    /* Set ccr1 */
    LL_TIM_OC_SetCompareCH1(TIM14, (100 - fanLevel) * fanPwmArr / 100);
}

void FanStop(void)
{
    /* Disable channel */
    LL_TIM_CC_DisableChannel(TIM14, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1);

    /* Disable Counter */
    LL_TIM_DisableCounter(TIM14);
    LL_TIM_DisableCounter(TIM3);
    
    /* Clear interrupt flag */
    LL_TIM_ClearFlag_CC1(TIM3);
}

/*
 * Set PWM mode & CCRx according to val
 * val: from 0(min) to 100(max)
 */
void FanSetLevel(uint16_t targetLevel)
{
    if (targetLevel > 100) {
        targetLevel = 100;
    }

    while (targetLevel > fanLevel) {
        fanLevel ++;
        LL_TIM_OC_SetCompareCH1(TIM14, (100 - fanLevel) * fanPwmArr / 100);
        if (fanLevel > 10) {
            HAL_Delay(100);
        }
    }
    while (targetLevel < fanLevel) {
        fanLevel --;
        LL_TIM_OC_SetCompareCH1(TIM14, (100 - fanLevel) * fanPwmArr / 100);
        if (fanLevel > 10) {
            HAL_Delay(100);
        }
    }
    fanLevel = targetLevel;

    LL_TIM_OC_SetCompareCH1(TIM14, (100 - fanLevel) * fanPwmArr / 100);
}

uint16_t FanGetLevel(void)
{
    return fanLevel;
}

