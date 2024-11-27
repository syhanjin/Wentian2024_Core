//
// Created by hanjin on 24-9-23.
//

#ifndef BLE_H
#define BLE_H
#include "main.h"

#define BLE_NAME "CORE"
#define BLE_AMDATA "FFAA0001"

#define BLE_CMD_LENGTH 8
#define BLE_CMD_BEGIN  0xAA
#define BLE_CMD_END    0xBB

#define BLE_RX_BUFFER_SIZE 48
#define BLE_W_CMD_TIMEOUT 10000

#define BLE_UART_HANDLE (&huart1)

typedef struct
{
    uint8_t buffer[BLE_RX_BUFFER_SIZE];
    uint8_t size;
} BLE_Rx_t;

typedef enum
{
    BLE_STATUS_INITIALIZING = 0U,
    BLE_STATUS_READY = 1U,
} BLE_Status_t;

typedef enum
{
    BLE_W_READY = 0U,
    BLE_W_BUSY = 1U,
    BLE_W_SUCCESS = 2U,
    BLE_W_ERROR = 3U,
} BLE_W_Status_t;

extern BLE_Rx_t BLE_rx_buffer;
extern BLE_Status_t BLE_Status;

uint8_t BLE_Init(void);
uint8_t BLE_Set_Name(char* name);
uint8_t BLE_Set_AMData(char* amData);

void BLE_RxCpltCallback(void);
void CMD_Handler(uint8_t cmd, uint8_t data[]);

#endif //BLE_H
