#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "main.h"
#include "mpu6050.h"
#include "ssd1306.h"

void DisplaySensorData(MPU6050_Data *data);
void DisplayString(uint8_t x, uint8_t page, const char *str);
void DisplayNumber(uint8_t x, uint8_t page, int num);
void DisplayFloat(uint8_t x, uint8_t page, float num, uint8_t decimal);

#endif
