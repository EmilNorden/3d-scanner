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
	LIDAR_OK,
	LIDAR_UART_ERROR,
	LIDAR_OUT_OF_RANGE,
	LIDAR_LOW_SNR,
	LIDAR_INVALID_CHECKSUM,
	LIDAR_LASER_NOT_ON,
	LIDAR_UNKNOWN_ERROR,
} lidar_status_t;

typedef enum {
	LIDAR_OFF,
	LIDAR_ON,
} lidar_state_t;

typedef struct {
	UART_HandleTypeDef huart;
	lidar_state_t state;
	uint8_t rx_buffer[10];
} lidar_t;

lidar_t lidar_new(UART_HandleTypeDef huart);
lidar_status_t lidar_turn_on(lidar_t* lidar);
lidar_status_t lidar_measure(lidar_t* lidar, int *out_distance);

#endif /* INC_LIDAR_H_ */
