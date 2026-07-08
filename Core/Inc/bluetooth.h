#include <stdint.h>
#include <mpu6050.h>
#include "usart.h"

// 自定义数据帧
#define FRAME_STX   0xAA
#define FRAME_ETX   0x55

// 指令码
#define CMD_SYNC_TIME   0x01
#define CMD_STEP_DATA   0x02
#define CMD_SENSOR_DATA 0x03
#define CMD_SET_ALARM   0x04
#define CMD_DEV_STATUS  0x05

// 环形接收缓冲区
#define BT_RX_BUF_SIZE  256
extern uint8_t bt_rx_buf[BT_RX_BUF_SIZE];

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void Bluetooth_Init(void);
void Bluetooth_SendSensorData(MPU6050_Data *data);
void Bluetooth_SendStepData(uint32_t steps);
void Bluetooth_ProcessReceived(void);
void Bluetooth_RxIdleCallback(void);  
void Bluetooth_SendSensorText(MPU6050_Data *data);
