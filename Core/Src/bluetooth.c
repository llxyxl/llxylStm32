#include "bluetooth.h"
#include "main.h"
#include "freertos_tasks.h"   
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "queue.h"

#define xQueueOverwriteSafe(queue, data) \
    do { \
        xQueueReset((queue)); \
        xQueueSend((queue), (data), 0); \
    } while(0)

static volatile uint8_t bt_rx_idle_flag = 0;

extern osMessageQId queueStepDataHandle;
extern osSemaphoreId semDisplayUpdateHandle;
extern osMutexId mutexUARTHandle;
extern UART_HandleTypeDef huart1;

uint8_t bt_rx_buf[BT_RX_BUF_SIZE];
static uint32_t bt_rx_last_pos = 0;
static uint8_t rx_state = 0;
static uint8_t rx_cmd = 0;
static uint8_t rx_len = 0;
static uint8_t rx_data[128];
static uint8_t rx_data_idx = 0;

static uint8_t CalculateFrameCHK(uint8_t cmd, uint8_t len, uint8_t *data);
static void ProcessSyncTime(uint8_t *data, uint8_t len);
static void ProcessStepData(uint8_t *data, uint8_t len);
static void ProcessSensorData(uint8_t *data, uint8_t len);
static void ProcessSetAlarm(uint8_t *data, uint8_t len);
static void ProcessDevStatus(uint8_t *data);
static void ParseReceivedByte(uint8_t byte);
static void UartLock(void);
static void UartUnLock(void);

void Bluetooth_Init(void)
{
    HAL_UART_Receive_DMA(&huart1, bt_rx_buf, BT_RX_BUF_SIZE);
    bt_rx_last_pos = 0;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

static void UartLock(void)
{
    osMutexWait(mutexUARTHandle, osWaitForever);
}
static void UartUnLock(void)
{
    osMutexRelease(mutexUARTHandle);
}

void Bluetooth_SendSensorData(MPU6050_Data *data)
{
    uint8_t frame[17];
    int16_t ax = (int16_t)(data->accel_x * 1000.0f);
    int16_t ay = (int16_t)(data->accel_y * 1000.0f);
    int16_t az = (int16_t)(data->accel_z * 1000.0f);
    int16_t gx = (int16_t)(data->gyro_x * 1000.0f);
    int16_t gy = (int16_t)(data->gyro_y * 1000.0f);
    int16_t gz = (int16_t)(data->gyro_z * 1000.0f);
    frame[0] = FRAME_STX;
    frame[1] = CMD_SENSOR_DATA;
    frame[2] = 12;
    memcpy(&frame[3], &ax, 2);
    memcpy(&frame[5], &ay, 2);
    memcpy(&frame[7], &az, 2);
    memcpy(&frame[9], &gx, 2);
    memcpy(&frame[11], &gy, 2);
    memcpy(&frame[13], &gz, 2);
    frame[15] = CalculateFrameCHK(frame[1], frame[2], &frame[3]); 
    frame[16] = FRAME_ETX;
    UartLock();
    HAL_UART_Transmit(&huart1, frame, 17, 100);
    UartUnLock();
}

void Bluetooth_SendStepData(uint32_t steps)
{
    uint8_t frame[9];
    frame[0] = FRAME_STX;
    frame[1] = CMD_STEP_DATA;
    frame[2] = 4;
    memcpy(&frame[3], &steps, 4);
    frame[7] = CalculateFrameCHK(frame[1], frame[2], &frame[3]);
    frame[8] = FRAME_ETX;
    UartLock();
    HAL_UART_Transmit(&huart1, frame, 9, 100);
    UartUnLock();
}

void Bluetooth_SendSensorText(MPU6050_Data *data)
{
    char buf[100];
    sprintf(buf, "AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f\r\n",
            data->accel_x, data->accel_y, data->accel_z,
            data->gyro_x, data->gyro_y, data->gyro_z);
    UartLock();
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    UartUnLock();
}

void Bluetooth_ProcessReceived(void)
{
    if (bt_rx_idle_flag == 0) return;
    bt_rx_idle_flag = 0;

    uint32_t current_pos = BT_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    while (bt_rx_last_pos != current_pos) {
        ParseReceivedByte(bt_rx_buf[bt_rx_last_pos]);
        bt_rx_last_pos = (bt_rx_last_pos + 1) % BT_RX_BUF_SIZE;
    }
}

void Bluetooth_RxIdleCallback(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    bt_rx_idle_flag = 1;
}

static uint8_t CalculateFrameCHK(uint8_t cmd, uint8_t len, uint8_t *data)
{
    uint8_t chk = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}

static void ParseReceivedByte(uint8_t data)
{
    switch (rx_state) {
        case 0:
            if (data == FRAME_STX) rx_state = 1;
            break;
        case 1:
            rx_cmd = data; rx_state = 2; break;
        case 2:
            rx_len = data; rx_data_idx = 0;
            rx_state = (rx_len > 0 && rx_len <= 128) ? 3 : 0;
            break;
        case 3:
            rx_data[rx_data_idx++] = data;
            if(rx_data_idx >= rx_len) rx_state = 4;
            break;
        case 4:
        {
            uint8_t recv_chk = data;
            uint8_t calc = CalculateFrameCHK(rx_cmd, rx_len, rx_data);
            if(calc == recv_chk)
            {
                switch(rx_cmd)
                {
                    case CMD_SYNC_TIME: ProcessSyncTime(rx_data, rx_len); break;
                    case CMD_STEP_DATA: ProcessStepData(rx_data, rx_len); break;
                    case CMD_SENSOR_DATA: ProcessSensorData(rx_data, rx_len); break;
                    case CMD_SET_ALARM: ProcessSetAlarm(rx_data, rx_len); break;
                    case CMD_DEV_STATUS: ProcessDevStatus(rx_data); break;
                }
            }
            rx_state = 0;
            break;
        }
        default: rx_state = 0; break;
    }
}

static void ProcessSyncTime(uint8_t *data, uint8_t len)
{
    if(len < 7) return;
    TimeData *t = GetGlobalTimeBuf();
    t->year  = (uint16_t)((data[0] << 8) | data[1]);
    t->month = data[2];
    t->day   = data[3];
    t->hour  = data[4];
    t->min   = data[5];
    t->sec   = data[6];
    osSemaphoreRelease(semDisplayUpdateHandle);
}

static void ProcessStepData(uint8_t *data, uint8_t len)
{
    if(len == 4)
    {
        uint32_t new_step;
        memcpy(&new_step, data, 4);
        xQueueOverwriteSafe(queueStepDataHandle, &new_step);
        osSemaphoreRelease(semDisplayUpdateHandle);
    }
    else
    {
        Bluetooth_SendStepData(GetGlobalStep());
    }
}

static void ProcessSensorData(uint8_t *data, uint8_t len)
{
    (void)data;(void)len;
}

static void ProcessSetAlarm(uint8_t *data, uint8_t len)
{
    if(len >= 2)
    {
        extern void SetAlarm(uint8_t h,uint8_t m);
        SetAlarm(data[0], data[1]);
    }
}

static void ProcessDevStatus(uint8_t *data)
{
    (void)data;
    uint8_t frame[32];
    TimeData *t = GetGlobalTimeBuf();
    uint32_t step = GetGlobalStep();
    frame[0] = FRAME_STX;
    frame[1] = CMD_DEV_STATUS;
    frame[2] = 10;
    memcpy(&frame[3], &t->hour, 1);
    memcpy(&frame[4], &t->min, 1);
    memcpy(&frame[5], &t->sec, 1);
    memcpy(&frame[6], &step, 4);
    frame[13] = CalculateFrameCHK(frame[1], frame[2], &frame[3]);
    frame[14] = FRAME_ETX;
    UartLock();
    HAL_UART_Transmit(&huart1, frame, 15, 100);
    UartUnLock();
}
