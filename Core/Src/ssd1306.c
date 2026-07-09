#include "ssd1306.h"
#include "font5x8.h"
#include <string.h>

// 128x64 分辨率对应 128列 × 8页 = 1024字节显存
static uint8_t ssd1306_vram[128 * 8];
static uint8_t cur_x = 0;
static uint8_t cur_y = 0;

extern I2C_HandleTypeDef hi2c1;

static uint8_t current_x = 0;
static uint8_t current_page = 0;

void ssd1306_WriteCommand(uint8_t cmd)
{
    uint8_t data[2] = {SSD1306_CMD, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, data, 2, 100);
}

void ssd1306_WriteData(uint8_t data)
{
    uint8_t buf[2] = {SSD1306_DATA, data};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, buf, 2, 100);
}

void ssd1306_WriteData_Buffer(uint8_t *data, uint16_t size)
{
    for(uint16_t i = 0; i < size; i++) 
	  {
        ssd1306_WriteData(data[i]);
    }
}

void ssd1306_Init(void)
{
    HAL_Delay(100);

    ssd1306_WriteCommand(0xAE);
    ssd1306_WriteCommand(0xD5);
    ssd1306_WriteCommand(0x80);
    ssd1306_WriteCommand(0xA8);
    ssd1306_WriteCommand(0x3F);
    ssd1306_WriteCommand(0xD3);
    ssd1306_WriteCommand(0x00);
    ssd1306_WriteCommand(0x40);

    ssd1306_WriteCommand(0x8D);
    ssd1306_WriteCommand(0x14);

    ssd1306_WriteCommand(0xA1);
    ssd1306_WriteCommand(0xC8);
    ssd1306_WriteCommand(0xDA);
    ssd1306_WriteCommand(0x12);
    ssd1306_WriteCommand(0x81);
    ssd1306_WriteCommand(0xCF);
    ssd1306_WriteCommand(0xD9);
    ssd1306_WriteCommand(0xF1);
    ssd1306_WriteCommand(0xDB);
    ssd1306_WriteCommand(0x30);
    ssd1306_WriteCommand(0x20);
    ssd1306_WriteCommand(0x00);
    ssd1306_WriteCommand(0xA4);
    ssd1306_WriteCommand(0xA6);
    ssd1306_WriteCommand(0xAF);

    ssd1306_Clear();
}

void ssd1306_Clear(void)
{
    memset(ssd1306_vram, 0, sizeof(ssd1306_vram));
    cur_x = 0;
    cur_y = 0;
}

void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    cur_x = x;
    cur_y = y;
}

void ssd1306_WriteChar(char ch)
{
    if(ch < 32 || ch > 126) ch = 32;
    const uint8_t *font_data = font5x8[ch - 32];
    for(uint8_t i = 0; i < 6; i++)
    {
        ssd1306_WriteData(font_data[i]);
    }
    current_x += 6;
    if(current_x > 121)
    {
        current_x = 0;
        current_page++;
        if(current_page > 7)
        {
            current_page = 0;
            ssd1306_Clear();
        }
        ssd1306_SetCursor(current_x, current_page);
    }
}

void ssd1306_WriteString(const char *str)
{
    extern const uint8_t font5x8[96][6]; // 使用字模数组
    while(*str)
    {
        // 超出宽度自动换行
        if(cur_x + 6 > 128) {
            cur_x = 0;
            cur_y++;
        }
        if(cur_y >= 8) break; // 超出底部停止绘制

        // 将字模数据写入显存对应页
        for(uint8_t i = 0; i < 6; i++)
        {
            ssd1306_vram[cur_y * 128 + cur_x + i] = font5x8[*str - ' '][i];
        }
        cur_x += 6;
        str++;
    }
}

void ssd1306_UpdateScreen(void)
{
    // 逐页写入，共8页
    for(uint8_t page = 0; page < 8; page++)
    {
        ssd1306_WriteCommand(0xB0 + page);   // 设置页地址
        ssd1306_WriteCommand(0x00);          // 列地址低4位
        ssd1306_WriteCommand(0x10);          // 列地址高4位
        
        // 连续写入当前页的128字节
        HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, 1, 
                         &ssd1306_vram[page * 128], 128, 100);
    }
}

// 亮度调节
void ssd1306_SetContrast(uint8_t val)
{
    ssd1306_WriteCommand(0x81);  // 对比度指令
    ssd1306_WriteCommand(val);  // 0~255
}
