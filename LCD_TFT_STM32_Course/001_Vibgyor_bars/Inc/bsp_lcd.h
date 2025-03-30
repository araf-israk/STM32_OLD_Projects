/*
 * bsp_lcd.h
 *
 *  Created on: Mar 1, 2025
 *      Author: arafi
 */

#ifndef BSP_LCD_H_
#define BSP_LCD_H_

#include "stm32f411xe.h"
#include "reg_util.h"
#include "ili9341_reg.h"

void BSP_LCD_Init(void);

typedef struct{
	uint16_t x1;
	uint16_t x2;
	uint16_t y1;
	uint16_t y2;
}lcd_area_t;

void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);
void lcd_set_display_area(lcd_area_t *area);
struct bsp_lcd;

#endif /* BSP_LCD_H_ */
