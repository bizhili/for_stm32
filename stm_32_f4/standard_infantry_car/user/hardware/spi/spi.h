#ifndef SPI_H
#define SPI_H
#include "main.h"
#include "stm32f4xx.h"
void SPI5Init(void);
void SPI5SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize);
void SPI5_DMA_Init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void SPI5_DMA_Enable(uint16_t ndtr);
extern void usart2_init(u32 bound);
extern u8 USART2_RX_BUF[32];
extern u16 USART2_RX_STA;
void SPI6Init(void);
void SPI6SetSpeedAndDataSize(uint16_t Speed, uint8_t DataSize);
void MySPI_SendData1(u8 da);
#endif
