/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program with UART interrupt-based frame reception
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h> // Required for memcpy
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * Frame structure received from ROS:
 * [0]   : 0xAA (start byte)
 * [1-4] : float linear velocity
 * [5-8] : float angular velocity
 * [9]   : 0x55 (stop byte)
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t header;    // 0xAA start byte
    float v_lin;       // Linear velocity from ROS
    float v_ang;       // Angular velocity from ROS
    uint8_t footer;    // 0x55 stop byte
} TelemetryFrame_t;
#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// UART reception variables
uint8_t rx_byte;                     // Single incoming byte (not used in current state machine)
uint8_t rx_buffer[10];               // Raw frame buffer (10 bytes total)
uint8_t rx_index = 0;                // State machine index
volatile uint8_t new_data_flag = 0;  // Flag indicating a valid frame has been received

TelemetryFrame_t rxFrame;            // Parsed frame structure
char msgBuffer[100];                 // Debug print buffer for UART2

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Redirect printf to UART2 (for debugging via terminal)
 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 0 */

/**
  * @brief  Application entry point
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    /* USER CODE BEGIN 2 */

    printf("Robot system started...\r\n");

    rx_index = 0;

    // Start UART interrupt reception (first byte = frame sync search)
    HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);

    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        if (new_data_flag == 1)
        {
            new_data_flag = 0;

            // Print received ROS command
            int len = sprintf(msgBuffer,
                              "ROS Cmd -> V_lin: %.3f, V_ang: %.3f\r\n",
                              rxFrame.v_lin,
                              rxFrame.v_ang);

            HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, len, 100);
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure voltage scaling */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure MSI oscillator */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure CPU, AHB and APB clocks */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/*
 * UART RX complete callback
 * Implements a simple state machine for frame decoding:
 * 1. Wait for start byte (0xAA)
 * 2. Receive remaining 9 bytes
 * 3. Validate stop byte (0x55)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // STATE 0: waiting for frame start byte
        if (rx_index == 0)
        {
            if (rx_buffer[0] == 0xAA)
            {
                rx_index = 1;

                // Receive the rest of the frame (9 bytes)
                HAL_UART_Receive_IT(&huart3, &rx_buffer[1], 9);
            }
            else
            {
                // Invalid byte, keep searching for start
                HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
            }
        }
        // STATE 1: full frame received (10 bytes total)
        else
        {
            // Validate stop byte
            if (rx_buffer[9] == 0x55)
            {
                // Copy raw data into structured format
                memcpy(&rxFrame, rx_buffer, 10);
                new_data_flag = 1;
            }

            // Reset state machine
            rx_index = 0;
            HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
        }
    }
}

/*
 * UART error callback
 * Recovers from overrun/framing errors and restarts reception
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        rx_index = 0;
        HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Error handler
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add debug printf here */
}
#endif
