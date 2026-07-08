#include "display.h"
#include <stdio.h>
#include <math.h>

// 在指定位置显示字符串
void DisplayString(uint8_t x, uint8_t page, const char *str)
{
    ssd1306_SetCursor(x, page);
    ssd1306_WriteString((char *)str);
}

// 在指定位置显示整数
void DisplayNumber(uint8_t x, uint8_t page, int num)
{
    char buf[16];
    sprintf(buf, "%d", num);
    DisplayString(x, page, buf);
}

// 在指定位置显示浮点数
void DisplayFloat(uint8_t x, uint8_t page, float num, uint8_t decimal)
{
    char buf[16];
    char format[8];
    sprintf(format, "%%.%df", decimal); // 生成格式字符串，如 "%.2f"
    sprintf(buf, format, num);
    DisplayString(x, page, buf);
}

// 传感器数据显示

void DisplaySensorData(MPU6050_Data *data)
{
    char buf[32];

    // 先清屏
    ssd1306_Clear();

    // 第一行（Page 0）：标题 
    DisplayString(0, 0, "MPU6050 Data");

    // 第二行（Page 1）：加速度 X
    DisplayString(0, 1, "AX:");
    DisplayFloat(30, 1, data->accel_x, 1);
    DisplayString(84, 1, "g");

    // 第三行（Page 2）：加速度 Y 
    DisplayString(0, 2, "AY:");
    DisplayFloat(30, 2, data->accel_y, 1);
    DisplayString(84, 2, "g");

    // 第四行（Page 3）：加速度 Z 
    DisplayString(0, 3, "AZ:");
    DisplayFloat(30, 3, data->accel_z, 1);
    DisplayString(84, 3, "g");

    // 第五行（Page 4）：陀螺仪 X 
    DisplayString(0, 4, "GX:");
    DisplayFloat(30, 4, data->gyro_x, 1);
    DisplayString(90, 4, "/s");

    // 第六行（Page 5）：陀螺仪 Y 
    DisplayString(0, 5, "GY:");
    DisplayFloat(30, 5, data->gyro_y, 1);
    DisplayString(90, 5, "/s");

    // 第七行（Page 6）：陀螺仪 Z 
    DisplayString(0, 6, "GZ:");
    DisplayFloat(30, 6, data->gyro_z, 1);
    DisplayString(90, 6, "/s");

    // 第八行（Page 7）：温度
    DisplayString(0, 7, "Temp:");
    DisplayFloat(42, 7, data->temp, 1);
    DisplayString(96, 7, "C");

    // 把内存里的画面一次性刷到屏幕
    ssd1306_UpdateScreen();
}
