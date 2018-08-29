/**
 ******************************************************************************
 * File Name          : led.c
 * Description        : This file provide the customize config to uart for indoor led module control
 ******************************************************************************
 *  Write based on HAL_UART and HAL_DMA APIs.
 *
 *  Set UART TX with DMA normal mode for sending data when needed.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "led_tbl.h"

#define REG_LED_CR1 (LED_UART->CR1)
#define REG_LED_TDR (LED_UART->TDR)
#define REG_LED_ISR (LED_UART->ISR)
#define REG_LED_ICR (LED_UART->ICR)
#define REG_LED_CR3 (LED_UART->CR3)

#define REG_LED_TX_DMA_CPAR (LED_TX_DMA->CPAR)
#define REG_LED_TX_DMA_CCR (LED_TX_DMA->CCR)
#define REG_LED_TX_DMA_CNDTR (LED_TX_DMA->CNDTR)
#define REG_LED_TX_DMA_CMAR (LED_TX_DMA->CMAR)

extern UART_HandleTypeDef LED_UART_HANDLE;

static uint8_t gu8Data[4];

static HAL_StatusTypeDef SendDataToLEDDriver(uint8_t len)
{
    /* direct control to register to enhance the efficiency */

    /* Check that a Tx process is not already ongoing */
    if (0x0 == (REG_LED_ISR & USART_ISR_TC) ) {
        /* if TC_BIT is low, means the last transmission is not complete */
        return HAL_BUSY;
    } else {

        REG_LED_TX_DMA_CCR &= ~DMA_CCR_EN;

        /* Configure DMA Channel data length */
        REG_LED_TX_DMA_CNDTR = len;

        /* Configure DMA Channel source address */
        REG_LED_TX_DMA_CMAR = (uint32_t)gu8Data;

        /* Configure DMA Channel destination address */
        /* pre-set in init func already */
        /* REG_LED_TX_DMA_CPAR = (uint32_t)&REG_LED_TDR; */

        /* enable DMA */
        REG_LED_TX_DMA_CCR |= DMA_CCR_EN;

        /* clear TC_BIT */
        REG_LED_ICR = USART_ICR_TCCF;

        /* Enable the DMA transfer for transmit request by setting the DMAT bit
           in the UART CR3 register */
        /* REG_LED_CR3 |= USART_CR3_DMAT; */

        return HAL_OK;
    }
}

void Drv_LED_Init(void)
{
    /* disable IRQ which are enabled by CubeMX created code by default */
    HAL_NVIC_DisableIRQ(LED_TX_DMA_IRQN);

    /* pre-set TXDMA settings, to reduce the cycle in write API */
    /* can be pre-set because CPAR will not change in TX direction from Memory to Peripheral */
    REG_LED_TX_DMA_CPAR = (uint32_t)&REG_LED_TDR;

    /* pre-set TXDMAEN, since the actual DMA will be triggered when Drv_CommPC_TxSend() is called */
    REG_LED_CR3 |= USART_CR3_DMAT;
}

HAL_StatusTypeDef Drv_LED_CtrlSet(EN_DISPLAY_ON_PERCENTAGE enPercent, EN_DISPLAY_DRIVING enDriv, EN_DISPLAY_SEGMODE enSegMode)
{
    gu8Data[0] = LED_CTRL_CMD;
    gu8Data[1] = (enPercent << 4) | (enDriv << 1) | enSegMode;

    return SendDataToLEDDriver(2);
}

HAL_StatusTypeDef Drv_LED_DisplayClear(void)
{
    gu8Data[0] = LED_ADDR_NUM;
    gu8Data[1] = LED_LETTER_NONE;
    gu8Data[2] = LED_LETTER_NONE;
    gu8Data[3] = LED_LETTER_NONE;

    return SendDataToLEDDriver(4);
}

HAL_StatusTypeDef Drv_LED_NumDisplay(uint8_t num)
{
    gu8Data[0] = LED_ADDR_NUM;
    gu8Data[1] = led_digit[ (num%10) ];
    gu8Data[2] = led_digit[ (num/10) ];

    return SendDataToLEDDriver(3);
}

HAL_StatusTypeDef Drv_LED_ErrCodeDisplay(uint8_t num)
{
    if (num>9) {
        return HAL_ERROR;
    }

    gu8Data[0] = LED_ADDR_NUM;
    gu8Data[1] = led_digit[ num ];
    gu8Data[2] = LED_LETTER_E;

    return SendDataToLEDDriver(3);
}

HAL_StatusTypeDef Drv_LED_ColorLEDCtrl(uint8_t red_on, uint8_t yellow_on, uint8_t green_on)
{
    gu8Data[0] = LED_ADDR_COLORLED;
    gu8Data[1] = red_on | (yellow_on << 1) | (green_on << 2);

    return SendDataToLEDDriver(2);
}
