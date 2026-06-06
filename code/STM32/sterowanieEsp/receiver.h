/*
 * receiver.h
 *
 *  Created on: Jan 22, 2026
 *      Author: ktocz
 */

#ifndef INC_RECEIVER_H_
#define INC_RECEIVER_H_
#include "main.h"

extern UART_HandleTypeDef huart3;
extern uint8_t rx_data;
extern volatile uint8_t robot_state;


void Receiver_Start(void);

#endif /* INC_RECEIVER_H_ */
