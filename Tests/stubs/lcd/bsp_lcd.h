#ifndef LCD_TEST_STUB_H
#define LCD_TEST_STUB_H
#include <stdint.h>
#include "fonts/fonts.h"
#define LCD_COLOR_BLACK 0u
#define LCD_COLOR_WHITE 0xFFFFFFFFu
#define LEFT_MODE 3u
void LCD_SetFont(sFONT *font);
void LCD_SetBackColor(uint32_t color);
void LCD_SetTextColor(uint32_t color);
void LCD_Clear(uint32_t color);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint32_t color);
void LCD_DisplayStringAt(uint16_t x, uint16_t y, uint8_t *text, unsigned mode);
#endif
