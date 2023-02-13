/*
 * laser.h
 *
 *  Created on: Feb 11, 2023
 *      Author: emilnorden
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "stm32f7xx_hal.h"

typedef enum {
	FALSE,
	TRUE
} success_t;

void lidar_init(UART_HandleTypeDef *huart);
success_t lidar_turn_on(void);
success_t lidar_turn_off(void);
success_t lidar_measure(int *out_distance);

#endif /* INC_LIDAR_H_ */
