#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include "cmsis_os.h"
#include "ssd1306.h"
#include "mpu6050.h"
#include "bluetooth.h"
#include <math.h>

// 时间数据结构
typedef struct 
{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} TimeData;

// 页面菜单枚举
typedef enum 
{
    PAGE_MAIN_MENU = 0,  // 主菜单列表页
    
    /* 1. 时间显示 */
    PAGE_TIME_MENU,      // 时间显示子菜单
    PAGE_TIME_NOW,       // 当前时间详情
    PAGE_DATE,           // 日期详情
    PAGE_STEP_VIEW,      // 步数统计详情
    
    /* 2. 传感器数据 */
    PAGE_SENSOR_MENU,    // 传感器子菜单
    PAGE_ATTITUDE,       // 姿态角度
    PAGE_ACCEL,          // 加速度
    PAGE_TEMP,           // 温度
    
    /* 3. 运动数据 */
    PAGE_SPORT_MENU,     // 运动数据子菜单
    PAGE_STEP_TODAY,     // 今日步数
    PAGE_DISTANCE,       // 运动距离
    PAGE_CALORIE,        // 卡路里估算
    
    /* 4. 设置 */
    PAGE_SETTING_MENU,   // 设置子菜单
    PAGE_TIME_SET,       // 时间设置
    PAGE_BT_PAIR,        // 蓝牙配对
    PAGE_BRIGHTNESS,     // 屏幕亮度
		PAGE_ALARM_SET,      // 新增：闹钟设置
    PAGE_FACTORY_RESET,  // 恢复出厂
    
    /* 5. 系统信息 */
    PAGE_SYSINFO_MENU,   // 系统信息子菜单
    PAGE_BATTERY,        // 电池电压
    PAGE_FW_VERSION,     // 固件版本
    PAGE_BT_MAC,         // 蓝牙MAC地址
    
    PAGE_MAX
} MenuPage;

// 菜单消息
typedef struct 
{
    int16_t delta;      // 编码器增量
    uint8_t button;     // 按键事件
} MenuMsg;

// 菜单状态
typedef struct 
{
    uint8_t current_item;
    uint8_t edit_mode;
} MenuState;

// 按键事件定义
#define BTN_NONE    0
#define BTN_SHORT   1
#define BTN_LONG    2

// 函数声明
void IncrementTime(TimeData *time);
uint8_t ReadEncoderButton(void);

// 外部全局接口
extern TimeData* GetGlobalTimeBuf(void);
extern uint32_t GetGlobalStep(void);
extern void SetAlarm(uint8_t hour, uint8_t min);
extern void StepDetection(MPU6050_Data *dat, uint32_t *step);

#endif
