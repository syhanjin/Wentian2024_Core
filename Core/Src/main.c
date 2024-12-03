/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "ble.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  // 移动动作组
  CHASSIS_FORWARD = 0x01U,
  CHASSIS_BACKWARD = 0x02U,
  CHASSIS_LEFT = 0x04U,
  CHASSIS_RIGHT = 0x08U,
  // 奇葩操作，旋转动作组
  CHASSIS_ANTICLOCKWISE = 0x03U,
  CHASSIS_CLOCKWISE = 0x0CU,
  // 预设动作组，四个动作只能同时执行一个
  CHASSIS_TO_SLOPE = 0x10U,
  CHASSIS_TO_TOP = 0x20U,
  CHASSIS_TO_BOTTOM = 0x40U,
  CHASSIS_TO_RIM = 0x80U,
} Chassis_State_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t cmd_rb[UPPER_CMD_LENGTH];
uint8_t cmd_chassis_tb[CHASSIS_CMD_LENGTH];

uint8_t chassis_state;

// float chassis_vx, chassis_vy, ///< 底盘速度
//       chassis_omega, ///< 底盘角速度
//       chassis_x, chassis_y; ///< 底盘位移
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Chassis_SendCMD(uint8_t cmd, uint8_t* data1, uint8_t* data2);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf retarget */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim1) {}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  if (huart == BLE_UART_HANDLE)
  {
    BLE_rx_buffer.size = Size;
    BLE_RxCpltCallback();
    HAL_UARTEx_ReceiveToIdle_IT(BLE_UART_HANDLE, BLE_rx_buffer.buffer, BLE_RX_BUFFER_SIZE);
  }
}

void Chassis_State_Handler(uint8_t state, uint8_t speed)
{
  if (state == chassis_state)
  {
    Chassis_SendCMD(0x77, 0, 0);
    return;
  }
  // 当之前存在正在执行的动作组，拒绝被状态消失打断
  if ((chassis_state & 0xF0) && (chassis_state | state) == chassis_state) return;
  uint8_t cmd = 0x07;
  float data1 = 0, data2 = 0;
  // 检测动作组变化，只对动作新增敏感
  if (((state | chassis_state) & 0xF0) != (chassis_state & 0xF0))
  {
    cmd = 0x07;
    if (state & CHASSIS_TO_SLOPE)
    {
      data1 = 100;
      data2 = 0;
    } else if (state & CHASSIS_TO_TOP)
    {
      data1 = 0;
      data2 = 150;
    } else if (state & CHASSIS_TO_BOTTOM)
    {
      data1 = 0;
      data2 = -150;
    } else if (state & CHASSIS_TO_RIM)
    {
      data1 = -100;
      data2 = 0;
    }
  } else
  {
    // TODO: 重新根据一些速度极限数据分配速度
    float vx = 0, vy = 0, omega = 0;
    if ((state & CHASSIS_ANTICLOCKWISE) == CHASSIS_ANTICLOCKWISE)
      omega += speed;
    else
    {
      if (state & CHASSIS_FORWARD)
        vy += speed;
      if (state & CHASSIS_BACKWARD)
        vy -= speed;
    }
    if ((state & CHASSIS_CLOCKWISE) == CHASSIS_CLOCKWISE)
      omega -= speed;
    else
    {
      if (state & CHASSIS_LEFT)
        vx -= speed;
      if (state & CHASSIS_RIGHT)
        vx += speed;
    }
    if (omega)
    {
      cmd = 0x04;
      data1 = 10 / 255.0 * omega;
    } else if (vx || vy)
    {
      cmd = 0x05;
      data1 = vx, data2 = vy;
    }
  }
  chassis_state = state;
  Chassis_SendCMD(cmd, (uint8_t*)&data1, (uint8_t*)&data2);
}

void CMD_Handler(uint8_t cmd, uint8_t data[4])
{
  switch (cmd)
  {
  case 0x05:
    // printf("%c%c%c%c\n", data[0], data[1], data[2], data[3]);
    Chassis_State_Handler(data[0], data[1]);
    break;
  default: ;
  }
}

void Chassis_SendCMD(uint8_t cmd, uint8_t* data1, uint8_t* data2)
{
  // 协议头
  cmd_chassis_tb[0] = 0x0F;
  // 指令位
  cmd_chassis_tb[1] = cmd;
  if (cmd != 0x02 && cmd != 0x03 && cmd != 0x77)
  {
    // 数据组1
    cmd_chassis_tb[2] = *(data1 + 0);
    cmd_chassis_tb[3] = *(data1 + 1);
    cmd_chassis_tb[4] = *(data1 + 2);
    cmd_chassis_tb[5] = *(data1 + 3);
    // 数据组2
    cmd_chassis_tb[6] = *(data2 + 0);
    cmd_chassis_tb[7] = *(data2 + 1);
    cmd_chassis_tb[8] = *(data2 + 2);
    cmd_chassis_tb[9] = *(data2 + 3);
  }
  // 协议尾
  cmd_chassis_tb[CHASSIS_CMD_LENGTH - 2] = 0xF0;
  // 校验和
  cmd_chassis_tb[CHASSIS_CMD_LENGTH - 1] = 0;
  for (int i = 0; i < CHASSIS_CMD_LENGTH - 1; i++)
    cmd_chassis_tb[CHASSIS_CMD_LENGTH - 1] += cmd_chassis_tb[i];

  // 发送指令
  // 在 115200 的波特率下发送 12 个字节大约需要 1.04 ms，在目前的情况下发送间隔 > 80ms，故不需要考虑多次发送占用问题
  HAL_UART_Transmit(CHASSIS_UART_HANDLE, cmd_chassis_tb, CHASSIS_CMD_LENGTH, 0xFFFF);
  // HAL_UART_Transmit_IT(CHASSIS_UART_HANDLE, cmd_chassis_tb, CHASSIS_CMD_LENGTH);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
#if DEBUG
  printf("Hello World!\n");
#endif
  BLE_Init();
#if DEBUG
  printf("Ble Ready!\n");
#endif
  Chassis_SendCMD(0x02, 0, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
