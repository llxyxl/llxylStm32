/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "freertos_tasks.h"
#include <stdio.h>   
#include "display.h"
#include <string.h>
#include <stdlib.h>
#include "tim.h" 
#include "gpio.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct QueueSensorBuf{
    MPU6050_Data data;
}QueueSensorBuf_t;
typedef struct QueueTimeBuf{
    TimeData data;
}QueueTimeBuf_t;
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 等效 xQueueOverwriteSafe，兼容多版本
#define xQueueOverwriteSafe(queue, data) \
    do { \
        xQueueReset((queue)); \
        xQueueSend((queue), (data), 0); \
    } while(0)
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 全局缓存，避免局部变量入队野指针
static TimeData g_timeBuf = {0};
static MPU6050_Data g_sensorBuf = {0};
static uint32_t g_stepTotal = 0;
static MenuPage g_curPage = PAGE_MAIN_MENU; // 开机默认进主菜单
static uint8_t g_menu_cursor = 0;          // 列表页当前光标位置
static uint8_t g_alarmHour = 0;
static uint8_t g_alarmMin = 0;
static uint8_t g_alarm_enabled = 1;    // 闹钟总开关，默认开启
static uint8_t g_alarm_ringing = 0;    // 闹钟是否正在响铃
static uint8_t g_alarm_beep_tick = 0;  // 间歇鸣叫计数器

static uint8_t g_brightness = 207;  // 屏幕亮度，默认0xCF对应207
static uint8_t g_edit_mode = 0;     // 设置页编辑模式：0=选择项，1=编辑数值
static uint8_t g_edit_index = 0;    // 当前编辑的选项索引
static uint16_t last_enc_cnt = 32768;
static int16_t  dir_accum = 0;          // 方向累加器
#define ENC_TRIGGER  4                 // 累计4个计数触发1格（四倍频对应1个定位）
// 距离、卡路里计算常量
#define STEP_PER_METER  0.7f        // 步长0.7米
#define CAL_PER_STEP    0.04f       // 每步约0.04千卡
#define FW_VERSION      "V1.0.0"    // 固件版本
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId taskTimeKeepHandle;
osThreadId taskSensorHandle;
osThreadId taskDisplayHandle;
osThreadId taskMenuHandle;
osThreadId taskBluetoothHandle;
osThreadId taskHwInitHandle;
osMessageQId queueSensorDataHandle;
osMessageQId queueTimeUpdateHandle;
osMessageQId queueMenuMsgHandle;
osMessageQId queueBtCmdHandle;
osMessageQId queueStepDataHandle;
osMutexId mutexI2CHandle;
osMutexId mutexUARTHandle;
osSemaphoreId semDisplayUpdateHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void StepDetection(MPU6050_Data *dat, uint32_t *step);
/* USER CODE END FunctionPrototypes */
void StartDefaultTask(void const * argument);
void vTask_TimeKeep(void const * argument);
void vTask_Sensor(void const * argument);
void vTask_Display(void const * argument);
void vTask_Menu(void const * argument);
void vTask_Bluetooth(void const * argument);
void vTaskHardwareInit(void const * arg);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
int16_t EncoderSimByButtons(void);
/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}



/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) 
{
		osMutexDef(mutexI2C);
		mutexI2CHandle = osMutexCreate(osMutex(mutexI2C));
		if (mutexI2CHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Mutex I2C Fail");
				while(1);
		}
		
		// 创建 UART 互斥锁
		osMutexDef(mutexUART);
		mutexUARTHandle = osMutexCreate(osMutex(mutexUART));
		if (mutexUARTHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Mutex UART Fail");
				while(1);
		}
		
		osSemaphoreDef(semDisplayUpdate);
		semDisplayUpdateHandle = osSemaphoreCreate(osSemaphore(semDisplayUpdate), 1);
		if (semDisplayUpdateHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("sem display Fail");
				while(1);
		}


		osThreadDef(taskDisplay, vTask_Display, osPriorityNormal, 0, 256);
		taskDisplayHandle = osThreadCreate(osThread(taskDisplay), NULL);
		if (taskDisplayHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("task diplay Fail");
				while(1);
		}
		
		// 创建时间更新队列
		queueTimeUpdateHandle = xQueueCreate(2, sizeof(QueueTimeBuf_t));
		if (queueTimeUpdateHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("queue Timeupdate Fail");
				while(1);
		}
		
		// 创建传感器数据队列
		queueSensorDataHandle = xQueueCreate(4, sizeof(QueueSensorBuf_t));
		if (queueSensorDataHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("queue sensor data Fail");
				while(1);
		}
		
		// 创建计步数据队列
		queueStepDataHandle = xQueueCreate(1, sizeof(uint32_t));
		if (queueStepDataHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Mqueue step data Fail");
				while(1);
		}
		
		// 创建菜单消息队列
		queueMenuMsgHandle = xQueueCreate(4, sizeof(MenuMsg));
		if (queueMenuMsgHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("queue menu msg Fail");
				while(1);
		}
		
		//蓝牙队列
		queueBtCmdHandle = xQueueCreate(4, 32);
		if (queueBtCmdHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("queue bt cmd Fail");
				while(1);
		}

		// 硬件初始化任务
		osThreadDef(taskHwInit, vTaskHardwareInit, osPriorityHigh+1, 0, 256);
		taskHwInitHandle = osThreadCreate(osThread(taskHwInit), NULL);
		if (taskHwInitHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("task hw init Fail");
				while(1);
		}
		
		// 时间任务
		osThreadDef(taskTimeKeep, vTask_TimeKeep, osPriorityHigh, 0, 128);
		taskTimeKeepHandle = osThreadCreate(osThread(taskTimeKeep), NULL);
		if (taskTimeKeepHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("task time keep Fail");
				while(1);
		}
		
		// 菜单任务
		osThreadDef(taskMenu, vTask_Menu, osPriorityNormal, 0, 128);
		taskMenuHandle = osThreadCreate(osThread(taskMenu), NULL);
		if (taskMenuHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("task menu Fail");
				while(1);
		}
		
		// 传感器任务
		osThreadDef(taskSensor, vTask_Sensor, osPriorityNormal, 0, 192);
		taskSensorHandle = osThreadCreate(osThread(taskSensor), NULL);
		if (taskSensorHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("task sensor Fail");
				while(1);
		}
		
		// 蓝牙任务
		osThreadDef(taskBluetooth, vTask_Bluetooth, osPriorityLow, 0, 192);
		taskBluetoothHandle = osThreadCreate(osThread(taskBluetooth), NULL);
		if (taskBluetoothHandle == NULL) 
		{
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("task blue tooth Fail");
				while(1);
		}
}

/* USER CODE BEGIN Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  for(;;)
  {
    osDelay(1000);
  }
}
/* USER CODE END Header_StartDefaultTask */

void vTask_TimeKeep(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    QueueTimeBuf_t timePack;
    static TimeData lastTime = {0};
    for (;;) {
        IncrementTime(&g_timeBuf);

        /* ========== 新增：闹钟检测逻辑 ========== */
        if(g_alarm_enabled && !g_alarm_ringing)
        {
            // 时分匹配且秒为0，整点触发闹钟，避免重复触发
            if(g_timeBuf.hour == g_alarmHour 
            && g_timeBuf.min == g_alarmMin 
            && g_timeBuf.sec == 0)
            {
                g_alarm_ringing = 1;
                g_alarm_beep_tick = 0;
                osSemaphoreRelease(semDisplayUpdateHandle); // 触发屏幕刷新提示
            }
        }

        // 闹钟响铃中：间歇鸣叫（响1秒停1秒）
        if(g_alarm_ringing)
        {
            g_alarm_beep_tick++;
            if(g_alarm_beep_tick % 2 == 1)
                Beep_Set(1);  // 奇数秒响
            else
                Beep_Set(0);  // 偶数秒停
            
            // 响铃超时：60秒后自动关闭
            if(g_alarm_beep_tick >= 60)
            {
                g_alarm_ringing = 0;
                Beep_Set(0);
            }
        }
        /* ======================================== */

        // 只有时间变化，才入队+触发刷新
        if(memcmp(&g_timeBuf, &lastTime, sizeof(TimeData)) != 0)
        {
            timePack.data = g_timeBuf;
            xQueueOverwriteSafe(queueTimeUpdateHandle, &timePack);
            osSemaphoreRelease(semDisplayUpdateHandle);
            lastTime = g_timeBuf;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/* 传感器任务：修复局部指针入队，新增计步调用 */
void vTask_Sensor(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    QueueSensorBuf_t sensorPack;
    static MPU6050_Data lastSensor = {0};
    static uint32_t lastStep = 0;
    for (;;) 
		{
        if (osMutexWait(mutexI2CHandle, pdMS_TO_TICKS(100)) == osOK) 
				{
            MPU6050_ReadAll(&g_sensorBuf);
            osMutexRelease(mutexI2CHandle);
        }
        
        StepDetection(&g_sensorBuf, &g_stepTotal);

        int dataChanged = 0;
        if(memcmp(&g_sensorBuf, &lastSensor, sizeof(MPU6050_Data)) != 0)
        {
            lastSensor = g_sensorBuf;
            dataChanged = 1;
        }
        if(g_stepTotal != lastStep)
        {
            lastStep = g_stepTotal;
            dataChanged = 1;
        }

        if(dataChanged)
        {
            sensorPack.data = g_sensorBuf;
            xQueueOverwriteSafe(queueSensorDataHandle, &sensorPack);
            xQueueOverwriteSafe(queueStepDataHandle, &g_stepTotal);
            osSemaphoreRelease(semDisplayUpdateHandle);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
    }
}

void vTask_Display(void const * argument)
{
    QueueTimeBuf_t timePack = {0};
    QueueSensorBuf_t sensorPack = {0};
    uint32_t steps = 0;

    // 启动时先读一次初始数据
    xQueueReceive((QueueHandle_t)queueTimeUpdateHandle, &timePack, 0);
    xQueueReceive((QueueHandle_t)queueSensorDataHandle, &sensorPack, 0);
    xQueueReceive((QueueHandle_t)queueStepDataHandle, &steps, 0);

    for (;;) 
		{
        // 永久等待刷新信号，内容不变不重绘，减少闪烁
        osSemaphoreWait(semDisplayUpdateHandle, osWaitForever);

        // 读取最新数据
        xQueueReceive((QueueHandle_t)queueTimeUpdateHandle, &timePack, 0);
        xQueueReceive((QueueHandle_t)queueSensorDataHandle, &sensorPack, 0);
        xQueueReceive((QueueHandle_t)queueStepDataHandle, &steps, 0);

        // 获取I2C互斥锁
        uint8_t lockTry = 0;
        while(osMutexWait(mutexI2CHandle, pdMS_TO_TICKS(20)) != osOK && lockTry < 3)
        {
            lockTry++;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if(lockTry >= 3)
            continue;

        ssd1306_Clear();
				
				switch(g_curPage)
				{
						// 主菜单（原有保留，略作优化）
						case PAGE_MAIN_MENU:
						{
								ssd1306_SetCursor(0, 0);
								ssd1306_WriteString("=== main menu ===");
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(g_menu_cursor==0 ? ">1.time display" : " 1.time display");
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString(g_menu_cursor==1 ? ">2.sensor data" : " 2.sensor data");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(g_menu_cursor==2 ? ">3.sport data" : " 3.sport data");
								ssd1306_SetCursor(0, 5);
								ssd1306_WriteString(g_menu_cursor==3 ? ">4.settings" : " 4.settings");
								ssd1306_SetCursor(0, 6);
								ssd1306_WriteString(g_menu_cursor==4 ? ">5.system info" : " 5.system info");
								break;
						}

						// 时间显示子菜单
						case PAGE_TIME_MENU:
						{
								ssd1306_SetCursor(0, 0);
								ssd1306_WriteString("=== time menu ===");
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(g_menu_cursor==0 ? ">current time" : " current time");
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString(g_menu_cursor==1 ? ">date" : " date");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(g_menu_cursor==2 ? ">step num" : " step num");
								break;
						}

						// 当前时间详情
						case PAGE_TIME_NOW:
						{
								char buf[32];
								sprintf(buf,"%02d:%02d:%02d", timePack.data.hour, timePack.data.min, timePack.data.sec);
								ssd1306_SetCursor(20, 3);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 日期页面
						case PAGE_DATE:
						{
								char buf[32];
								sprintf(buf, "%04d-%02d-%02d", timePack.data.year, timePack.data.month, timePack.data.day);
								ssd1306_SetCursor(16, 3);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 步数统计页面
						case PAGE_STEP_VIEW:
						{
								char buf[32];
								ssd1306_SetCursor(0, 1);
								ssd1306_WriteString("total steps:");
								sprintf(buf, "%u", steps);
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(buf);
								float dist = steps * STEP_PER_METER / 1000.0f;
								sprintf(buf, "dist: %.2f km", dist);
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 传感器子菜单
						case PAGE_SENSOR_MENU:
						{
								ssd1306_SetCursor(0, 0);
								ssd1306_WriteString("= sensor menu =");
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(g_menu_cursor==0 ? ">attitude" : " attitude");
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString(g_menu_cursor==1 ? ">accel" : " accel");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(g_menu_cursor==2 ? ">temp" : " temp");
								break;
						}

						// 姿态角度页面
						case PAGE_ATTITUDE:
						{
								char buf[32];
								ssd1306_SetCursor(0, 1);
								sprintf(buf, "Pitch: %.1f", sensorPack.data.pitch);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 3);
								sprintf(buf, "Roll : %.1f", sensorPack.data.roll);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 5);
								sprintf(buf, "Yaw  : %.1f", sensorPack.data.yaw);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 加速度页面
						case PAGE_ACCEL:
						{
								char buf[32];
								ssd1306_SetCursor(0, 1);
								sprintf(buf, "AX: %.2f g", sensorPack.data.accel_x);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 3);
								sprintf(buf, "AY: %.2f g", sensorPack.data.accel_y);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 5);
								sprintf(buf, "AZ: %.2f g", sensorPack.data.accel_z);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 温度页面
						case PAGE_TEMP:
						{
								char buf[32];
								ssd1306_SetCursor(20, 3);
								sprintf(buf, "Temp: %.1f C", sensorPack.data.temp);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 运动数据子菜单
						case PAGE_SPORT_MENU:
						{
								ssd1306_SetCursor(0, 0);
								ssd1306_WriteString("== sport menu ==");
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(g_menu_cursor==0 ? ">today steps" : " today steps");
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString(g_menu_cursor==1 ? ">distance" : " distance");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(g_menu_cursor==2 ? ">calorie" : " calorie");
								break;
						}

						// 今日步数
						case PAGE_STEP_TODAY:
						{
								char buf[32];
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString("today steps:");
								sprintf(buf, "%u steps", steps);
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 运动距离
						case PAGE_DISTANCE:
						{
								char buf[32];
								float dist = steps * STEP_PER_METER / 1000.0f;
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString("sport distance:");
								sprintf(buf, "%.2f km", dist);
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 卡路里估算
						case PAGE_CALORIE:
						{
								char buf[32];
								float cal = steps * CAL_PER_STEP;
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString("burn calorie:");
								sprintf(buf, "%.1f kcal", cal);
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 设置子菜单
						case PAGE_SETTING_MENU:
						{
								ssd1306_SetCursor(0, 0);
								ssd1306_WriteString("=== settings ===");
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(g_menu_cursor==0 ? ">time set" : " time set");
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString(g_menu_cursor==1 ? ">bt pair" : " bt pair");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(g_menu_cursor==2 ? ">brightness" : " brightness");
								ssd1306_SetCursor(0, 5);
								ssd1306_WriteString(g_menu_cursor==3 ? ">alarm set" : " alarm set"); 
								ssd1306_SetCursor(0, 6);
								ssd1306_WriteString(g_menu_cursor==4 ? ">factory reset" : " factory reset");
								break;
						}

						// 时间设置页面
						case PAGE_TIME_SET:
						{
								char buf[32];
								ssd1306_SetCursor(0, 0);
								ssd1306_WriteString("=== time set ===");
								// 三个选项：时、分、秒
								sprintf(buf, "%s hour : %02d", g_edit_index==0?">":" ", g_timeBuf.hour);
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(buf);
								sprintf(buf, "%s min  : %02d", g_edit_index==1?">":" ", g_timeBuf.min);
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString(buf);
								sprintf(buf, "%s sec  : %02d", g_edit_index==2?">":" ", g_timeBuf.sec);
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(buf);
								
								if(g_edit_mode) {
										ssd1306_SetCursor(0, 6);
										ssd1306_WriteString("editing...");
								}
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 蓝牙配对页面
						case PAGE_BT_PAIR:
						{
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString("bt pairing...");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString("name: HC-05");
								ssd1306_SetCursor(0, 5);
								ssd1306_WriteString("pin : 1234");
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 屏幕亮度调节
						case PAGE_BRIGHTNESS:
						{
								char buf[32];
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString("brightness:");
								sprintf(buf, "%d %%", (g_brightness * 100) / 255);
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(buf);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("rotate adjust");
								break;
						}
						
						// 闹钟设置页面
						case PAGE_ALARM_SET:
						{
								char buf[32];
								ssd1306_SetCursor(0, 0);
								ssd1306_WriteString("=== alarm set ===");
								
								sprintf(buf, "%s hour : %02d", g_edit_index==0?">":" ", g_alarmHour);
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(buf);
								
								sprintf(buf, "%s min  : %02d", g_edit_index==1?">":" ", g_alarmMin);
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString(buf);
								
								sprintf(buf, "%s status: %s", g_edit_index==2?">":" ", g_alarm_enabled?"ON ":"OFF");
								ssd1306_SetCursor(0, 5);
								ssd1306_WriteString(buf);
								
								if(g_edit_mode) {
										ssd1306_SetCursor(0, 6);
										ssd1306_WriteString("editing...");
								}
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 恢复出厂
						case PAGE_FACTORY_RESET:
						{
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString("factory reset?");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString("short press confirm");
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press cancel");
								break;
						}

						// 系统信息子菜单
						case PAGE_SYSINFO_MENU:
						{
								ssd1306_SetCursor(0, 0);
								ssd1306_WriteString("= system info =");
								ssd1306_SetCursor(0, 2);
								ssd1306_WriteString(g_menu_cursor==0 ? ">battery" : " battery");
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString(g_menu_cursor==1 ? ">fw version" : " fw version");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(g_menu_cursor==2 ? ">bt mac" : " bt mac");
								break;
						}

						// 电池电压
						case PAGE_BATTERY:
						{
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString("battery: --.- V");
								ssd1306_SetCursor(0, 5);
								ssd1306_WriteString("(adc not init)");
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 固件版本
						case PAGE_FW_VERSION:
						{
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString("firmware ver:");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString(FW_VERSION);
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						// 蓝牙MAC
						case PAGE_BT_MAC:
						{
								ssd1306_SetCursor(0, 3);
								ssd1306_WriteString("bt mac:");
								ssd1306_SetCursor(0, 4);
								ssd1306_WriteString("--:--:--:--");
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}

						default:
						{
								ssd1306_SetCursor(20, 3);
								ssd1306_WriteString("wait...");
								ssd1306_SetCursor(0, 7);
								ssd1306_WriteString("long press back");
								break;
						}
				}
				ssd1306_UpdateScreen();
				osMutexRelease(mutexI2CHandle);
    }
}
/* 菜单任务：消费菜单队列、切换页面 */
void vTask_Menu(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 每个页面对应的上级页面
    const MenuPage menu_parent[PAGE_MAX] = {
        [PAGE_MAIN_MENU]    = PAGE_MAIN_MENU,
        [PAGE_TIME_MENU]    = PAGE_MAIN_MENU,
        [PAGE_TIME_NOW]     = PAGE_TIME_MENU,
        [PAGE_DATE]         = PAGE_TIME_MENU,
        [PAGE_STEP_VIEW]    = PAGE_TIME_MENU,
        
        [PAGE_SENSOR_MENU]  = PAGE_MAIN_MENU,
        [PAGE_ATTITUDE]     = PAGE_SENSOR_MENU,
        [PAGE_ACCEL]        = PAGE_SENSOR_MENU,
        [PAGE_TEMP]         = PAGE_SENSOR_MENU,
        
        [PAGE_SPORT_MENU]   = PAGE_MAIN_MENU,
        [PAGE_STEP_TODAY]   = PAGE_SPORT_MENU,
        [PAGE_DISTANCE]     = PAGE_SPORT_MENU,
        [PAGE_CALORIE]      = PAGE_SPORT_MENU,
        
        [PAGE_SETTING_MENU] = PAGE_MAIN_MENU,
        [PAGE_TIME_SET]     = PAGE_SETTING_MENU,
        [PAGE_BT_PAIR]      = PAGE_SETTING_MENU,
        [PAGE_BRIGHTNESS]   = PAGE_SETTING_MENU,
				[PAGE_ALARM_SET]    = PAGE_SETTING_MENU,
        [PAGE_FACTORY_RESET]= PAGE_SETTING_MENU,
        
        [PAGE_SYSINFO_MENU] = PAGE_MAIN_MENU,
        [PAGE_BATTERY]      = PAGE_SYSINFO_MENU,
        [PAGE_FW_VERSION]   = PAGE_SYSINFO_MENU,
        [PAGE_BT_MAC]       = PAGE_SYSINFO_MENU,
    };
    
    // 每个列表页的菜单项总数
    const uint8_t menu_item_count[PAGE_MAX] = {
        [PAGE_MAIN_MENU]    = 5,
        [PAGE_TIME_MENU]    = 3,
        [PAGE_SENSOR_MENU]  = 3,
        [PAGE_SPORT_MENU]   = 3,
        [PAGE_SETTING_MENU] = 5,
        [PAGE_SYSINFO_MENU] = 3,
        // 内容页/设置页项数为0
        [PAGE_TIME_NOW]     = 0, [PAGE_DATE] = 0, [PAGE_STEP_VIEW] = 0,
        [PAGE_ATTITUDE]     = 0, [PAGE_ACCEL] = 0, [PAGE_TEMP] = 0,
        [PAGE_STEP_TODAY]   = 0, [PAGE_DISTANCE] = 0, [PAGE_CALORIE] = 0,
        [PAGE_TIME_SET]     = 0, [PAGE_BT_PAIR] = 0, [PAGE_BRIGHTNESS] = 0,
				[PAGE_ALARM_SET] = 0,  [PAGE_FACTORY_RESET]= 0, [PAGE_BATTERY] = 0, 
				[PAGE_FW_VERSION] = 0, [PAGE_BT_MAC] = 0,
    };



    for (;;) {
        uint8_t refresh = 0;
        // EC11 读取差分增量
        uint16_t raw_cnt = TIM3->CNT;
        int16_t encoder_delta = (int16_t)(raw_cnt - last_enc_cnt);
        last_enc_cnt = raw_cnt;
        
        uint8_t btn_event = ReadEncoderButton();
			  
				// 临时用按键模拟编码器
        //int16_t encoder_delta = EncoderSimByButtons();

        // 旋转时屏蔽按键，双重防误触
        if(abs(encoder_delta) >= 2) {
            btn_event = BTN_NONE;
        }

        // ========== 方向累加防抖逻辑 ==========
        if(encoder_delta > 0) {
            if(dir_accum < 0) dir_accum = 0;  // 反向直接清零，过滤抖动
            dir_accum += encoder_delta;
        } else if(encoder_delta < 0) {
            if(dir_accum > 0) dir_accum = 0;
            dir_accum += encoder_delta;
        }
				

        // ========== 1. 处理编码器旋转 ==========
        if(g_edit_mode)
        {
            // 编辑模式：调整数值
            if(dir_accum >= ENC_TRIGGER) {
                dir_accum = 0;  // 触发后清零
                switch(g_curPage)
                {
                    case PAGE_TIME_SET:
                        if(g_edit_index == 0) {
                            g_timeBuf.hour = (g_timeBuf.hour + 1 + 24) % 24;
                        } else if(g_edit_index == 1) {
                            g_timeBuf.min = (g_timeBuf.min + 1 + 60) % 60;
                        } else if(g_edit_index == 2) {
                            g_timeBuf.sec = (g_timeBuf.sec + 1 + 60) % 60;
                        }
                        refresh = 1;
                        break;
                    case PAGE_BRIGHTNESS:
                        if(g_brightness < 255 - 10) g_brightness += 10;
                        else g_brightness = 255;
                        refresh = 1;
                        break;
										case PAGE_ALARM_SET:
												if(g_edit_index == 0) {
														g_alarmHour = (g_alarmHour + 1 + 24) % 24;
												} else if(g_edit_index == 1) {
														g_alarmMin = (g_alarmMin + 1 + 60) % 60;
												} else if(g_edit_index == 2) {
														g_alarm_enabled = 1; // 加键 → 开启闹钟
												}
												refresh = 1;
												break;
                    default: break;
                }
            } else if(dir_accum <= -ENC_TRIGGER) {
                dir_accum = 0;
                switch(g_curPage)
                {
                    case PAGE_TIME_SET:
                        if(g_edit_index == 0) {
                            g_timeBuf.hour = (g_timeBuf.hour - 1 + 24) % 24;
                        } else if(g_edit_index == 1) {
                            g_timeBuf.min = (g_timeBuf.min - 1 + 60) % 60;
                        } else if(g_edit_index == 2) {
                            g_timeBuf.sec = (g_timeBuf.sec - 1 + 60) % 60;
                        }
                        refresh = 1;
                        break;
                    case PAGE_BRIGHTNESS:
                        if(g_brightness > 10) g_brightness -= 10;
                        else g_brightness = 0;
                        refresh = 1;
                        break;
										case PAGE_ALARM_SET:
												if(g_edit_index == 0) {
														g_alarmHour = (g_alarmHour - 1 + 24) % 24;
												} else if(g_edit_index == 1) {
														g_alarmMin = (g_alarmMin - 1 + 60) % 60;
												} else if(g_edit_index == 2) {
														g_alarm_enabled = 0; // 减键 → 关闭闹钟
												}
												refresh = 1;
												break;
                    default: break;
                }
            }
        }
        else
        {
            // 非编辑模式：列表页移动光标
            uint8_t item_cnt = menu_item_count[g_curPage];
            if(item_cnt > 0)
            {
                if(dir_accum >= ENC_TRIGGER) {
                    dir_accum = 0;
                    // 顺时针：光标下移
                    if(g_menu_cursor < item_cnt - 1) {
                        g_menu_cursor++;
                        refresh = 1;
                    }
                } else if(dir_accum <= -ENC_TRIGGER) {
                    dir_accum = 0;
                    // 逆时针：光标上移
                    if(g_menu_cursor > 0) {
                        g_menu_cursor--;
                        refresh = 1;
                    }
                }
            }
        }

        // ========== 2. 处理短按 ==========
        if(btn_event == BTN_SHORT)
        {
						if(g_alarm_ringing)
						{
								g_alarm_ringing = 0;
								Beep_Set(0);
								refresh = 1;
						}
            else if(g_edit_mode)
            {
                if(g_curPage == PAGE_TIME_SET) {
                    if(g_edit_index < 2) {
                        g_edit_index++;
                    } else {
                        g_edit_mode = 0;
                        g_edit_index = 0;
                    }
                    refresh = 1;
                }
								/* ========== 新增：闹钟设置页 循环切光标 ========== */
								else if(g_curPage == PAGE_ALARM_SET) {
										// 0=小时 → 1=分钟 → 0=小时，循环往复，不退出编辑
										g_edit_index = (g_edit_index + 1) % 3;
										refresh = 1;
								}
            }
            else
            {
                if(g_curPage == PAGE_FACTORY_RESET) {
                    g_timeBuf.hour = 12;
                    g_timeBuf.min = 0;
                    g_timeBuf.sec = 0;
                    g_stepTotal = 0;
                    g_brightness = 207;
                    g_curPage = PAGE_MAIN_MENU;
                    g_menu_cursor = 0;
                    refresh = 1;
                }
                else {
                    switch(g_curPage)
                    {
                        case PAGE_MAIN_MENU:
                            switch(g_menu_cursor) {
                                case 0: g_curPage = PAGE_TIME_MENU; break;
                                case 1: g_curPage = PAGE_SENSOR_MENU; break;
                                case 2: g_curPage = PAGE_SPORT_MENU; break;
                                case 3: g_curPage = PAGE_SETTING_MENU; break;
                                case 4: g_curPage = PAGE_SYSINFO_MENU; break;
                            }
                            g_menu_cursor = 0;
                            refresh = 1;
                            break;
                        case PAGE_TIME_MENU:
                            switch(g_menu_cursor) {
                                case 0: g_curPage = PAGE_TIME_NOW; break;
                                case 1: g_curPage = PAGE_DATE; break;
                                case 2: g_curPage = PAGE_STEP_VIEW; break;
                            }
                            refresh = 1;
                            break;
                        case PAGE_SENSOR_MENU:
                            switch(g_menu_cursor) {
                                case 0: g_curPage = PAGE_ATTITUDE; break;
                                case 1: g_curPage = PAGE_ACCEL; break;
                                case 2: g_curPage = PAGE_TEMP; break;
                            }
                            refresh = 1;
                            break;
                        case PAGE_SPORT_MENU:
                            switch(g_menu_cursor) {
                                case 0: g_curPage = PAGE_STEP_TODAY; break;
                                case 1: g_curPage = PAGE_DISTANCE; break;
                                case 2: g_curPage = PAGE_CALORIE; break;
                            }
                            refresh = 1;
                            break;
                        case PAGE_SETTING_MENU:
                            switch(g_menu_cursor) {
                                case 0: 
                                    g_curPage = PAGE_TIME_SET; 
                                    g_edit_mode = 1;
                                    g_edit_index = 0; 
                                    break;
                                case 1: g_curPage = PAGE_BT_PAIR; break;
                                case 2: 
                                    g_curPage = PAGE_BRIGHTNESS; 
                                    g_edit_mode = 1;
                                    break;
																case 3:
																		g_curPage = PAGE_ALARM_SET;
																		g_edit_mode = 1;
																		g_edit_index = 0;
																		break;
                                case 4: g_curPage = PAGE_FACTORY_RESET; break;
                            }
                            refresh = 1;
                            break;
                        case PAGE_SYSINFO_MENU:
                            switch(g_menu_cursor) {
                                case 0: g_curPage = PAGE_BATTERY; break;
                                case 1: g_curPage = PAGE_FW_VERSION; break;
                                case 2: g_curPage = PAGE_BT_MAC; break;
                            }
                            refresh = 1;
                            break;
                        default: break;
                    }
                }
            }
        }

        // ========== 3. 处理长按 ==========
        else if(btn_event == BTN_LONG)
        {
            if(g_edit_mode) {
                g_edit_mode = 0;
                g_edit_index = 0;
            }
            MenuPage parent = menu_parent[g_curPage];
            if(parent != g_curPage) {
                g_curPage = parent;
                g_menu_cursor = 0;
                refresh = 1;
            }
        }

        if(refresh) {
            osSemaphoreRelease(semDisplayUpdateHandle);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

/* 蓝牙任务：100ms周期，定时上报设备数据 */
void vTask_Bluetooth_HEX(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t count = 0;
    for (;;) {
        Bluetooth_ProcessReceived();
        // 1秒一次上报传感器
        if (++count % 10 == 0) {
            Bluetooth_SendSensorData(&g_sensorBuf);
            Bluetooth_SendStepData(g_stepTotal);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void vTask_Bluetooth(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t count = 0;
    for (;;) {
        Bluetooth_ProcessReceived();
        // 1秒一次上报，使用明文输出
        if (++count % 10 == 0) {
            Bluetooth_SendSensorText(&g_sensorBuf);
            // 步数也拼成明文发送
            char step_buf[32];
            sprintf(step_buf, "Step: %u\r\n", g_stepTotal);
            osMutexWait(mutexUARTHandle, osWaitForever);
            HAL_UART_Transmit(&huart1, (uint8_t*)step_buf, strlen(step_buf), 100);
            osMutexRelease(mutexUARTHandle);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}


void vTaskHardwareInit(void const * arg)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    // 复位计数器初始值
    __HAL_TIM_SET_COUNTER(&htim3, 32768);
	
		// 立刻非阻塞取走1个令牌，最终信号量初始状态 = 0
		osSemaphoreWait(semDisplayUpdateHandle, 0);
    g_timeBuf.hour = 12;
    g_timeBuf.min = 30;
    g_timeBuf.sec = 0;
    g_stepTotal = 1024;

    QueueTimeBuf_t initTime = {.data = g_timeBuf};
		// 仅操作已创建的时间队列
    
		xQueueOverwriteSafe((QueueHandle_t)queueTimeUpdateHandle, &initTime);
		
    QueueSensorBuf_t initSen = {.data = g_sensorBuf};
    xQueueOverwriteSafe(queueSensorDataHandle, &initSen);
    xQueueOverwriteSafe(queueStepDataHandle, &g_stepTotal);

    osSemaphoreRelease(semDisplayUpdateHandle);
    vTaskDelay(pdMS_TO_TICKS(100));
    vTaskDelete(NULL);
}

/**
 * @brief  用PA6/PA7连接模拟编码器输出
 * @retval int16_t 增量值：正数=顺时针，负数=逆时针，0=无操作
 * @note   按下一次返回 ±4，和EC11四倍频一格的数值一致，原有阈值完全兼容
 */
int16_t EncoderSimByButtons(void)
{
    // 按键消抖与连发状态
    static uint8_t  key_up_last = 1;     // PA7：逆时针/上移
    static uint32_t key_up_tick = 0;
    static uint8_t  key_up_repeat = 0;
    static uint32_t key_up_repeat_tick = 0;

    static uint8_t  key_down_last = 1;   // PA6：顺时针/下移
    static uint32_t key_down_tick = 0;
    static uint8_t  key_down_repeat = 0;
    static uint32_t key_down_repeat_tick = 0;

    int16_t ret = 0;
    uint32_t now = HAL_GetTick();

    // ========== 读取电平 ==========
    uint8_t key_up_now  = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
    uint8_t key_down_now = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

    // ========== 逆时针键（PA7）处理 ==========
    if(key_up_now != key_up_last) {
        key_up_last = key_up_now;
        key_up_tick = now;
        key_up_repeat = 0;
    }
    // 消抖通过：稳定按下
    else if(key_up_now == GPIO_PIN_RESET && (now - key_up_tick) > 30) {
        if(key_up_repeat == 0) {
            // 首次按下触发
            ret -= 4;
            key_up_repeat = 1;
            key_up_repeat_tick = now + 500; // 500ms后开启连发
        } else if(now >= key_up_repeat_tick) {
            // 长按连发：每100ms触发一次
            ret -= 4;
            key_up_repeat_tick = now + 100;
        }
    }

    // ========== 顺时针键（PA6）处理 ==========
    if(key_down_now != key_down_last) {
        key_down_last = key_down_now;
        key_down_tick = now;
        key_down_repeat = 0;
    }
    else if(key_down_now == GPIO_PIN_RESET && (now - key_down_tick) > 30) {
        if(key_down_repeat == 0) {
            ret += 4;
            key_down_repeat = 1;
            key_down_repeat_tick = now + 500;
        } else if(now >= key_down_repeat_tick) {
            ret += 4;
            key_down_repeat_tick = now + 100;
        }
    }

    return ret;
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 全局变量对外导出，供蓝牙、业务文件调用
TimeData* GetGlobalTimeBuf(void){return &g_timeBuf;}
uint32_t GetGlobalStep(void){return g_stepTotal;}
void SetAlarm(uint8_t h,uint8_t m){g_alarmHour = h;g_alarmMin = m;}
/* USER CODE END Application */
