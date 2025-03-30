/*
 * ws2812-spi.h
 *
 *  Created on: Mar 12, 2024
 *      Author: arafi
 */

#ifndef INC_WS2812_SPI_H_
#define INC_WS2812_SPI_H_


#define WS2812_NUM_LEDS 24
#define WS2812_SPI_HANDLE hspi1

#define WS2812_RESET_PULSE 60
#define WS2812_BUFFER_SIZE (WS2812_NUM_LEDS * 24 + WS2812_RESET_PULSE)

extern SPI_HandleTypeDef WS2812_SPI_HANDLE;
extern uint8_t ws2812_buffer[];

void ws2812_init(void);
void ws2812_send_spi(void);
void ws2812_pixel(uint16_t led_no, uint8_t r, uint8_t g, uint8_t b);
void ws2812_pixel_all(uint8_t r, uint8_t g, uint8_t b);


#endif /* INC_WS2812_SPI_H_ */
