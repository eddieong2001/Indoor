/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_TBL_H
#define __LED_TBL_H
/* Includes ------------------------------------------------------------------*/

const uint8_t led_digit[] = {
    0x3F, //0
    0x06, //1
    0x5B, //2
    0x4F, //3
    0x66, //4
    0x6D, //5
    0x7D, //6
    0x07, //7
    0x7F, //8
    0x6F, //9
};

#define LED_LETTER_E (0x79)

#define LED_LETTER_NONE (0x0)

#define LED_CTRL_CMD (0x18)

#define LED_ADDR_NUM (0x08)

#define LED_ADDR_COLORLED (0x48)

#endif  /* __LED_TBL_H */
