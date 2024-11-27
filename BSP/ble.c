//
// Created by hanjin on 24-9-23.
//

#include "ble.h"

#include <stdio.h>
#include <string.h>
#include "usart.h"


BLE_Rx_t BLE_rx_buffer;
BLE_Status_t BLE_Status = BLE_STATUS_INITIALIZING;
BLE_W_Status_t BLE_W_Status = BLE_W_READY;

uint8_t BLE_Send_CMD(char* cmd)
{
    const HAL_StatusTypeDef ret = HAL_UART_Transmit(BLE_UART_HANDLE, (uint8_t*)cmd, strlen(cmd), 1000);
    if (ret == HAL_OK) return 0;
    else return 1;
}

/**
 * @brief 向蓝牙模块发送写入AT指令
 * @param cmd 指令 以AT+开头 以\r\n结尾
 * @return 0 成功 1 发送失败 2 其他指令正在等待回复，指令发送已取消 3 指令返回ERROR 4 超时
 */
uint8_t BLE_Send_W_CMD(char* cmd)
{
    if (BLE_W_Status == BLE_W_BUSY) return 2; // 其他指令等待回复中
    BLE_W_Status = BLE_W_BUSY;
    // printf("CMD: %s\n", cmd);
    const uint8_t ret = BLE_Send_CMD(cmd);
    if (ret) return ret;
    // 等待指令返回
    uint32_t tick = HAL_GetTick();
    while (BLE_W_Status == BLE_W_BUSY && HAL_GetTick() - tick <= BLE_W_CMD_TIMEOUT);
    if (HAL_GetTick() - tick > BLE_W_CMD_TIMEOUT)
    {
        BLE_W_Status = BLE_W_READY;
        return 4;
    }
    if (BLE_W_Status == BLE_W_ERROR) return 3;
    BLE_W_Status = BLE_W_READY;
    return 0;
}

/**
 * @brief 初始化蓝牙模块
 * @return .
 *  - 0x1x 重启蓝牙模块失败
 *  - 0x2x 设置蓝牙名称失败
 *  - 0x3x 设置AMData失败
 *  x: 与BLE_Send_W_CMD返回值相同
 */
uint8_t BLE_Init(void)
{
    // 开始接收数据
    HAL_UARTEx_ReceiveToIdle_IT(BLE_UART_HANDLE, BLE_rx_buffer.buffer, BLE_RX_BUFFER_SIZE);
    // 执行初始化
    uint8_t ret;
    BLE_Status = BLE_STATUS_INITIALIZING;
    ret = BLE_Send_W_CMD("AT+REBOOT=1\r\n"); // 重启蓝牙模块
    if (ret) return 0x10 | ret;
    // printf("REBOOT OK\n");
    while (BLE_Status != BLE_STATUS_READY); // 等待初始化
    ret = BLE_Set_Name(BLE_NAME);
    if (ret) return 0x20 | ret;
    // printf("NAME OK\n");
    ret = BLE_Set_AMData(BLE_AMDATA);
    if (ret) return 0x30 | ret;
    // printf("AMDATA OK\n");
    return 0;
}


uint8_t BLE_Set_Name(char* name)
{
    char cmd[128];
    sprintf(cmd, "AT+NAME=%s\r\n", name);
    return BLE_Send_W_CMD(cmd);
}

uint8_t BLE_Set_AMData(char* amData)
{
    char cmd[128];
    sprintf(cmd, "AT+AMDATA=%s\r\n", amData);
    return BLE_Send_W_CMD(cmd);
}

void BLE_Rx_Error_Handler(void) {}

__weak void CMD_Handler(uint8_t cmd, uint8_t data[]) {}

/**
 * @brief 蓝牙串口接收数据回调，处理接收到的信息
 * @attention 上位机发送的指令以 0xFF 开头，第二个字节为指令的类型 后面的内容可以是参数
 */
void BLE_RxCpltCallback(void)
{
    // 需要限制字符串长度，否则会导致后面strcmp出错
    BLE_rx_buffer.buffer[BLE_rx_buffer.size] = '\0';
    // printf("**%s*%d*\n", BLE_rx_buffer.buffer, BLE_rx_buffer.size);
    if (BLE_rx_buffer.buffer[0] == '+') // 读取响应
    {
        if (BLE_rx_buffer.size == 8 && strcmp(BLE_rx_buffer.buffer, "+READY\r\n") == 0)
        {
            printf("BLE_READY\r\n");
            BLE_Status = BLE_STATUS_READY;
        }
    } else if (BLE_rx_buffer.buffer[0] == BLE_CMD_BEGIN &&
        BLE_rx_buffer.buffer[BLE_CMD_LENGTH - 2] == BLE_CMD_END) // 上位机发送指令
    {
        // for (uint16_t i = 0; i < BLE_rx_buffer.size; i++)printf("%c", BLE_rx_buffer.buffer[i]);
        uint8_t sum = 0;
        for (int i = 0; i < BLE_CMD_LENGTH - 1; i++) sum += BLE_rx_buffer.buffer[i];
        // printf("BLE_CMD%#.2x\n,%d:%d", BLE_rx_buffer.buffer[1], sum, BLE_rx_buffer.buffer[BLE_CMD_LENGTH - 1]);
        if (sum == BLE_rx_buffer.buffer[BLE_CMD_LENGTH - 1])
        {
            switch (BLE_rx_buffer.buffer[1])
            {
            case 0x00: // 重启指令
                NVIC_SystemReset();
            default:
                CMD_Handler(BLE_rx_buffer.buffer[1], BLE_rx_buffer.buffer + 2);
            }
        }
    } else
    {
        if (BLE_W_Status == BLE_W_BUSY)
        {
            // printf("BLE_W_BUSY\r\n");
            if (!strcmp(BLE_rx_buffer.buffer, "OK\r\n")) BLE_W_Status = BLE_W_READY;
            else
            {
                if (!strcmp(BLE_rx_buffer.buffer, "ERROR\r\n"))
                    BLE_W_Status = BLE_W_ERROR;
                BLE_Rx_Error_Handler();
            }
        } else
        {
            BLE_Rx_Error_Handler();
        }
    }
}
