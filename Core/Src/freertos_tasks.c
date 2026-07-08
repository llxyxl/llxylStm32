#include "freertos_tasks.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>

// 滤波常量
#define STEP_ALPHA 0.25f
#define STEP_MIN_INTERVAL 200
#define STEP_OFFSET 0.1f

// 静态计步缓存
static float mag_filter = 0.0f;
static uint32_t last_step_tick = 0;
static float max_mag = 1.2f;
static float min_mag = 0.8f;

extern TIM_HandleTypeDef htim1;

void IncrementTime(TimeData *time) {
    time->sec++;
    if (time->sec >= 60) { time->sec = 0; time->min++; }
    if (time->min >= 60) { time->min = 0; time->hour++; }
    if (time->hour >= 24) { time->hour = 0; time->day++; }
}

uint8_t ReadEncoderButton(void)
{
    static uint8_t  last_stable_level = 1;
    static uint32_t level_stable_tick = 0;
    static uint32_t press_start_tick  = 0;
    static uint8_t  is_stable_pressed = 0;
    uint8_t ret = BTN_NONE;

    uint8_t cur_level = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

    // 电平变化：重置稳定计时，直接返回，不做任何判断
    if(cur_level != last_stable_level)
    {
        last_stable_level = cur_level;
        level_stable_tick = HAL_GetTick();
        return BTN_NONE;
    }

    // 电平持续不足20ms，视为抖动/误触，忽略
    if(HAL_GetTick() - level_stable_tick < 20)
    {
        return BTN_NONE;
    }

    // 稳定低电平， 确认按下
    if(cur_level == GPIO_PIN_RESET && is_stable_pressed == 0)
    {
        is_stable_pressed = 1;
        press_start_tick = HAL_GetTick();
    }
    // 稳定高电平， 确认松开，判断长短按
    else if(cur_level == GPIO_PIN_SET && is_stable_pressed == 1)
    {
        is_stable_pressed = 0;
        uint32_t dur = HAL_GetTick() - press_start_tick;
        if(dur > 1000)
            ret = BTN_LONG;
        else if(dur > 50)   // 按下总时长超过50ms才算短按
            ret = BTN_SHORT;
        // 小于50ms的全部视为误触，直接丢弃
    }

    return ret;
}

void StepDetection(MPU6050_Data *dat, uint32_t *step)
{
    float mag = sqrtf(dat->accel_x*dat->accel_x + dat->accel_y*dat->accel_y + dat->accel_z*dat->accel_z);
    // IIR低通滤波
    mag_filter = STEP_ALPHA * mag_filter + (1 - STEP_ALPHA) * mag;
    // 更新最大最小值，动态阈值
    if(mag_filter > max_mag) max_mag = mag_filter;
    if(mag_filter < min_mag) min_mag = mag_filter;
    float threshold = (max_mag + min_mag)/2.0f + STEP_OFFSET;
    uint32_t now = HAL_GetTick();
    // 峰值检测+步间隔防抖
    if(mag_filter > threshold && (now - last_step_tick) > STEP_MIN_INTERVAL)
    {
        *step += 1;
        last_step_tick = now;
        // 重置极值，防止阈值漂移
        max_mag = mag_filter;
        min_mag = mag_filter;
    }
}
