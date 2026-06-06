/*
 * receiver.c
 *
 *  Created on: Jan 22, 2026
 *      Author: ktocz
 */

// ---------------------------------------------------

// RECEIVER NA HUART3

// ---------------------------------------------------
#include "receiver.h"

uint8_t rx_data = 'a';
volatile uint8_t robot_state = 'a';

void Receiver_Start(void)
{
    HAL_UART_Receive_IT(&huart3, &rx_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_data == '0' || rx_data == '1' || rx_data == '2' || rx_data == '3')
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            robot_state = rx_data;
        }

        HAL_UART_Receive_IT(&huart3, &rx_data, 1);
    }
}
