#ifndef LED_H
#define LED_H
#include "main.h"
#define OLED_DC_Pin GPIO_Pin_9
#define OLED_DC_GPIO_Port GPIOB
#define LED_R_Pin GPIO_Pin_3
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_Pin_7
#define LED_G_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_Pin_10
#define OLED_RST_GPIO_Port GPIOB
void led_configuration(void);

extern void led_green_on(void);
extern void led_green_off(void);
extern void led_green_toggle(void);

extern void led_red_on(void);
extern void led_red_off(void);
extern void led_red_toggle(void);

extern void flow_led_on(uint16_t num);
extern void flow_led_off(uint16_t num);
extern void flow_led_toggle(uint16_t num);


#include "spi.h"
#include "stm32f4xx_conf.h"

#define RCC_APB2Periph_OLED_PORT    RCC_APB2Periph_GPIOA
#define OLED_PORT                   GPIOA

#define OLED_DC_PIN     GPIO_Pin_9
#define OLED_DC_LOW     GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define OLED_DC_HIGH    GPIO_SetBits(GPIOB,GPIO_Pin_9)

#define OLED_RST_PIN    GPIO_Pin_10
#define OLED_RST_LOW    GPIO_ResetBits(GPIOB,GPIO_Pin_10)
#define OLED_RST_HIGH   GPIO_SetBits(GPIOB,GPIO_Pin_10)

#define OLED_CLK_PIN    GPIO_Pin_3
#define OLED_DATA_PIN   GPIO_Pin_7

void SPI_OLED_Init(void);
void OLED_Init(void);

#define Max_Column      128
#define Max_Row         64

#define X_WIDTH         128
#define Y_WIDTH         64

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH     6
#define VHAR_SIZE_HIGHT     12

#define OLED_CMD_Set()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
#define OLED_CMD_Clr()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)

#define OLED_RST_Set()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)
#define OLED_RST_Clr()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET)

typedef enum
{
    Pen_Clear = 0x00,
    Pen_Write = 0x01,
    Pen_Inversion = 0x02,
}Pen_Typedef;


/* function define */
void oled_init(void);
void oled_write_byte(uint8_t dat, uint8_t cmd);
void oled_display_on(void);
void oled_display_off(void);
void oled_refresh_gram(void);
void oled_clear(Pen_Typedef pen);
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen);
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen);
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr);
void oled_shownum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode, uint8_t len);
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr);
void oled_printf(uint8_t row, uint8_t col, const char *fmt,...);
void oled_LOGO(void);

//ledfont


#endif
