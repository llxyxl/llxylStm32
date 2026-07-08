#ifndef __SSD1306_H
#define __SSD1306_H

#include "main.h"
#include "i2c.h"

#define SSD1306_ADDR   0x78
#define SSD1306_CMD    0x00
#define SSD1306_DATA   0x40

void ssd1306_Init(void);
void ssd1306_Clear(void);
void ssd1306_WriteData(uint8_t data);
void ssd1306_SetCursor(uint8_t x, uint8_t page);
void ssd1306_WriteChar(char ch);
void ssd1306_WriteString(const char *str);
void ssd1306_UpdateScreen(void);

#endif
