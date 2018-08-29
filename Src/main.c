
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ----------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include <stdarg.h>
#include "led.h"
#include "ir.h"
#include "temperature.h"
#include "comm_inout.h"
#include "fan_ctrl.h"
#include "step_ctrl.h"
/* END Includes ------------*/

#define IN_OUT_TEST
#define FAN_TEST

/* Private variables -----------------------*/
ADC_HandleTypeDef hadc;
CRC_HandleTypeDef hcrc;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
/* Private variables -----------------------*/

void UART_PRINT(const char * format, ... )
{
    static char strBuff[256];
    va_list ap;
    int n;

    va_start(ap, format);
    n = vsnprintf ((char*)strBuff, 256, format, ap);
    va_end(ap);

    HAL_UART_Transmit(&huart2, (uint8_t*)strBuff, n, 100);
}

/* Private function prototypes -------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* Private function prototypes -------------------*/

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
    HAL_StatusTypeDef ret;
    ST_IR_DATA *pstIRData = NULL;

    uint8_t i;
    //uint8_t ir_get_en = 0;

    //int16_t s16TempCPIn;
    //int16_t s16TempCPOut;
    //int16_t s16TempEnv;

    uint32_t tickstart;
    uint32_t systick_ms;

    ST_COMMIO_DATA stCommTxData;
    ST_COMMIO_DATA *pstCommRxData;

    uint8_t manual_dir_flag = 0;
    uint8_t manual_dir_ccw = 0;
    uint8_t step_pos;
    uint32_t tick_manual_dir;


    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC_Init();
    MX_TIM3_Init();
    MX_TIM14_Init();
    MX_TIM15_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_CRC_Init();

    UART_PRINT("Indoor main entered\r\n");

    Drv_LED_Init();
    Drv_LED_DisplayClear();
    HAL_Delay(5);
    //Drv_LED_CtrlSet(EN_DISPLAY_ON_15, EN_DISPLAY_DRIVING_8, EN_DISPLAY_SEG_8);
    Drv_LED_CtrlSet(EN_DISPLAY_ON_6, EN_DISPLAY_DRIVING_4, EN_DISPLAY_SEG_8);
    HAL_Delay(5);

    Drv_IR_Init();
    Drv_IR_Receive_DMA_Start();

    Drv_Temp_ADC_Enable();

    if (HAL_OK != Drv_CommIO_Init()) {
        _Error_Handler(__FILE__, __LINE__);
    }

    //Power-on trigger buzzer
    HAL_GPIO_WritePin(GPIO_OUT_Buzzer_GPIO_Port, GPIO_OUT_Buzzer_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO_OUT_Buzzer_GPIO_Port, GPIO_OUT_Buzzer_Pin, GPIO_PIN_RESET);

    FanInit(20);
    FanStart();

    Drv_LED_ColorLEDCtrl(1, 0, 0);
    HAL_Delay(330);
    Drv_LED_ColorLEDCtrl(0, 1, 0);
    HAL_Delay(330);
    Drv_LED_ColorLEDCtrl(0, 0, 1);
    HAL_Delay(330);
    Drv_LED_ColorLEDCtrl(0, 0, 0);
    HAL_Delay(330);
    Drv_LED_NumDisplay(88);
    HAL_Delay(330);  
    Drv_LED_DisplayClear();

    //Fan-direction alignment:
    for (i = 0; i < 8; ++i) {
        step_ccw(10);
    } 
    step_pos = 0;


    /* Infinite loop */
    while (1)
    {
        //System time: 0~99msec
        systick_ms = HAL_GetTick();
        systick_ms = systick_ms % 10;
                
        if(systick_ms==0){//TDM: slot1: check IR message.
            ret = Drv_IR_GetData(&pstIRData);
            if (HAL_OK == ret) {
                /* for dir_manual_en, it will be sent repeatly when the key is pressed */
                if (pstIRData->dir_manual_en) {
                    UART_PRINT("IR manual dir received, control step motor\r\n");
                    if (0 == manual_dir_flag) {
                        manual_dir_flag = 1;
                        tick_manual_dir = HAL_GetTick();
                    } else {
                        if ((HAL_GetTick() - tick_manual_dir) < 500) {
                            if (0 == manual_dir_ccw) {
                                step_cw(10);
                                step_pos += 10;
                                if (step_pos == 80) {
                                    manual_dir_ccw = 1;
                                }
                                UART_PRINT("CW 10 stpes\r\n");
                            } else {
                                step_ccw(10);
                                step_pos -= 10;
                                if (step_pos == 0) {
                                    manual_dir_ccw = 0;
                                }
                                UART_PRINT("CCW 10 stpes\r\n");
                            }
                            tick_manual_dir = HAL_GetTick();
                        } else {
                            manual_dir_flag = 0;
                        }
                    }
                } else {
                    //trigger buzzer
                    HAL_GPIO_WritePin(GPIO_OUT_Buzzer_GPIO_Port, GPIO_OUT_Buzzer_Pin, GPIO_PIN_SET);

                    UART_PRINT("-------------------\r\n");
                    UART_PRINT("IR_GetData OK\r\n");
                    UART_PRINT("-------------------\r\n");
                    UART_PRINT("set_temp = %d\r\n", pstIRData->set_temp);
                    UART_PRINT("body_sens = %d\r\n", pstIRData->body_sens);
                    UART_PRINT("curtime = %s %02d:%02d\r\n", (pstIRData->curtime_AMPM)?"AM":"PM", pstIRData->curtime_hour, pstIRData->curtime_min);
                    UART_PRINT("power_en = %d\r\n", pstIRData->power_en);
                    UART_PRINT("sleep_en = %d\r\n", pstIRData->sleep_en);
                    UART_PRINT("ABCode = %d\r\n", pstIRData->ABCode);
                    UART_PRINT("ontime En = %s, %s %02d:%02d\r\n", (pstIRData->ontime_en)?"ON":"OFF", (pstIRData->ontime_AMPM)?"AM":"PM",
                            pstIRData->ontime_hour, pstIRData->ontime_min);
                    UART_PRINT("offtime En = %s, %s %02d:%02d\r\n", (pstIRData->offtime_en)?"ON":"OFF", (pstIRData->offtime_AMPM)?"AM":"PM",
                            pstIRData->offtime_hour, pstIRData->offtime_min);
                    UART_PRINT("hour_off_en = %d, hour = %d\r\n", pstIRData->hour_off_en, pstIRData->hour_off_hour);
                    UART_PRINT("Function, warm = %d, cool = %d, dry = %d, fan = %d, auto = %d\r\n",
                            pstIRData->warm_en, pstIRData->cool_en, pstIRData->dry_en, pstIRData->fan_en, pstIRData->auto_en);
                    UART_PRINT("airclean_en = %d\r\n", pstIRData->airclean_en);
                    UART_PRINT("dir auto = %d, manual = %d\r\n", pstIRData->dir_auto_en, pstIRData->dir_manual_en);
                    UART_PRINT("auto_temp up = %d, down = %d\r\n", pstIRData->auto_temp_up, pstIRData->auto_temp_down);
                    UART_PRINT("fan auto = %d, high = %d, mid = %d, low = %d\r\n", pstIRData->fan_auto_en, pstIRData->fan_high_en,
                            pstIRData->fan_mid_en, pstIRData->fan_low_en);

                    HAL_Delay(10);
                    Drv_LED_NumDisplay(pstIRData->set_temp);
                    HAL_GPIO_WritePin(GPIO_OUT_Buzzer_GPIO_Port, GPIO_OUT_Buzzer_Pin, GPIO_PIN_RESET);

                    if(pstIRData->power_en==1){//power_on
                        Drv_LED_ColorLEDCtrl(0, 0, 1);
                        HAL_Delay(33);  

                        /* control step motor -----------------*/
                        if(pstIRData->dir_auto_en==1){
    
                        }else{//pstIRData->dir_manual_en==1
                            //Fan-direction middle position:
                            for (i = 0; i < 6; ++i) {
                                step_cw(10);
                            } 
                            step_pos = 6;
                        }
                        /*End of control step motor -----------*/

                        /*control fan_speed -------------------*/
                        if(pstIRData->fan_auto_en==1){
    
                        }else{
                            if(pstIRData->fan_high_en==1){
                                FanSetLevel(80);//91//78
                            }else if(pstIRData->fan_mid_en==1){
                                FanSetLevel(70);//86//75
                            }else{//pstIRData->fan_low_en==1
                                FanSetLevel(60);//80//70
                            }
                        }
                        /*End of control fan speed-------------*/
    
                        /* Send command to outdoor unit-------------------------------------------------------*/
                        stCommTxData.u16CmdResp = 1;
                        
                        if(pstIRData->cool_en==1){
                            stCommTxData.u16Param1 = 1;
                        }else if(pstIRData->warm_en==1){
                            stCommTxData.u16Param1 = 2;
                        }else{//pstIRData->dry_en==1
                            stCommTxData.u16Param1 = 3;
                        }
                        stCommTxData.u16Param2 = 150;//outdoor compressor speed
                        stCommTxData.u16Param3 = 58;//outdoor fan speed
    
                        //send out and readback check */
                        tickstart = HAL_GetTick();
                        do {
                            ret = Drv_CommIO_TxSend(&stCommTxData);
                            if ((HAL_GetTick() - tickstart) > 1000) {
                                UART_PRINT("IN_OUT_TEST TxSend timeout\r\n");
                                break;
                            }
                        } while (HAL_BUSY == ret);
                        UART_PRINT("stCommTxData.u16CRC = 0x%x\r\n", stCommTxData.u16CRC);
    
                        ////enable RX to read back
                        //Drv_CommIO_EnableRx(EN_Block);
                        //tickstart = HAL_GetTick();
                        //do {
                        //    ret = Drv_CommIO_RxRead(&pstCommRxData);
                        //    if ((HAL_GetTick() - tickstart) > 1000) {
                        //        UART_PRINT("IN_OUT_TEST RxRead timeout\r\n");
                        //        break;
                        //    }
                        //} while (HAL_BUSY == ret);
    
                        //check the readback value 
    
    
                        /* End of sned command to outdoor unit------------------------------------------------*/
                        
                    }else{//power_off, pstIRData->power_en==0
                        //Indoor stop process:
                        FanSetLevel(20);//FanStop();
                        //Fan-direction alignment:
                        for (i = 0; i < 8; ++i) {
                            step_ccw(10);
                        } 
                        step_pos = 0;
                        Drv_LED_DisplayClear();
                        HAL_Delay(100);  

                        //Outdoor stop process:
                        /* Send command to outdoor unit-------------------------------------------------------*/
                        stCommTxData.u16CmdResp = 0;
                        
                        if(pstIRData->cool_en==1){
                            stCommTxData.u16Param1 = 1;
                        }else if(pstIRData->warm_en==1){
                            stCommTxData.u16Param1 = 2;
                        }else{//pstIRData->dry_en==1
                            stCommTxData.u16Param1 = 3;
                        }
                        stCommTxData.u16Param2 = 50;//outdoor compressor speed
                        stCommTxData.u16Param3 = 20;//outdoor fan speed
    
                        //send out and readback check */
                        tickstart = HAL_GetTick();
                        do {
                            ret = Drv_CommIO_TxSend(&stCommTxData);
                            if ((HAL_GetTick() - tickstart) > 1000) {
                                UART_PRINT("IN_OUT_TEST TxSend timeout\r\n");
                                break;
                            }
                        } while (HAL_BUSY == ret);
    
                        ////enable RX to read back
                        //Drv_CommIO_EnableRx(EN_Block);
                        //tickstart = HAL_GetTick();
                        //do {
                        //    ret = Drv_CommIO_RxRead(&pstCommRxData);
                        //    if ((HAL_GetTick() - tickstart) > 1000) {
                        //        UART_PRINT("IN_OUT_TEST RxRead timeout\r\n");
                        //        break;
                        //    }
                        //} while (HAL_BUSY == ret);
    
                        //check the readback value 
        
                        /* End of sned command to outdoor unit------------------------------------------------*/


                    }
                }
            } else if (HAL_BUSY == ret) {
                /* UART_PRINT("IR_GetData Busy\r\n"); */
                if (manual_dir_flag) {
                    if ((HAL_GetTick() - tick_manual_dir) >= 500) {
                        manual_dir_flag = 0;
                    }
                }
            } else {
                UART_PRINT("IR_GetData Error\r\n");
                Drv_IR_Error_Recover();
            }            
            HAL_Delay(1);
        }else if(systick_ms==5){//TDM: slot2: checking state.

        }else{
            //Monitor Temp.:
        }
    }

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

    ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_9;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_15;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_CRCEx_Init(&hcrc) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 479;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 3;
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

    TIM_OC_InitTypeDef sConfigOC;

    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 11;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 0;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_TIM_MspPostInit(&htim14);

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig;
    TIM_IC_InitTypeDef sConfigIC;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim15.Instance = TIM15;
    htim15.Init.Prescaler = 0x12BF;
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.Period = 0x1000;
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
    sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
    sSlaveConfig.TriggerFilter = 3;
    if (HAL_TIM_SlaveConfigSynchronization(&htim15, &sSlaveConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 3;
    if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 1200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_AUTOBAUDRATE_INIT;
    huart1.AdvancedInit.AutoBaudRateEnable = UART_ADVFEATURE_AUTOBAUDRATE_ENABLE;
    huart1.AdvancedInit.AutoBaudRateMode = UART_ADVFEATURE_AUTOBAUDRATE_ON0X55FRAME;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 19200;
    huart3.Init.WordLength = UART_WORDLENGTH_9B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_ODD;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel2_3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    /* DMA1_Channel4_5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_OUT_Buzzer_Pin|GPIO_OUT_ION_En_Pin|GPIO_OUT_WaterPump_En_Pin|GPIO_OUT_UV_En_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_OUT_LN2003_IN3_Pin|GPIO_OUT_LN2003_IN2_Pin|GPIO_OUT_LN2003_IN1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIO_OUT_LN2003_IN0_GPIO_Port, GPIO_OUT_LN2003_IN0_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : GPIO_EXTI10_Force_Btn_Pin GPIO_EXTI3_WaterPump_Detect_Pin */
    GPIO_InitStruct.Pin = GPIO_EXTI10_Force_Btn_Pin|GPIO_EXTI3_WaterPump_Detect_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : GPIO_OUT_Buzzer_Pin GPIO_OUT_ION_En_Pin GPIO_OUT_WaterPump_En_Pin GPIO_OUT_UV_En_Pin */
    GPIO_InitStruct.Pin = GPIO_OUT_Buzzer_Pin|GPIO_OUT_ION_En_Pin|GPIO_OUT_WaterPump_En_Pin|GPIO_OUT_UV_En_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : GPIO_OUT_LN2003_IN3_Pin GPIO_OUT_LN2003_IN2_Pin GPIO_OUT_LN2003_IN1_Pin */
    GPIO_InitStruct.Pin = GPIO_OUT_LN2003_IN3_Pin|GPIO_OUT_LN2003_IN2_Pin|GPIO_OUT_LN2003_IN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : GPIO_OUT_LN2003_IN0_Pin */
    GPIO_InitStruct.Pin = GPIO_OUT_LN2003_IN0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_OUT_LN2003_IN0_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    UART_PRINT("ERROR @%s line %d\r\n", file, line);
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
