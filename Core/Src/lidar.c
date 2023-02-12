/*
 * lidar.c
 *
 *  Created on: Feb 11, 2023
 *      Author: emilnorden
 */

#include "lidar.h"
#include <string.h>

#define LIDAR_ERROR_HEADER 0x0E

typedef enum {
	LIDAR_ERROR_NONE = 0,
	LIDAR_ERROR_CHECKSUM = 0x81,
	LIDAR_ERROR_COMMAND_NOT_FOUND = 0x82,
	LIDAR_ERROR_OUT_OF_RANGE = 0x83,
	LIDAR_ERROR_INVALID_PARAMS = 0x84,
	LIDAR_ERROR_LASER_NOT_ON = 0x85,
	LIDAR_ERROR_LOW_SNR = 0x89,
	LIDAR_ERROR_WRONG_COMMAND_HEADER = 0x8B,
	// Added after own observations:
	LIDAR_ERROR_LASER_ALREADY_ON = 0x8D,
} lidar_error_code_t;


static lidar_error_code_t get_lidar_error(lidar_t* lidar) {
	if(lidar->rx_buffer[0] != LIDAR_ERROR_HEADER) {
		return LIDAR_ERROR_NONE;
	}

	return lidar->rx_buffer[1];
}

lidar_t lidar_new(UART_HandleTypeDef huart) {
	lidar_t lidar;
	lidar.huart = huart;
	lidar.state = LIDAR_OFF;

	return lidar;
}

lidar_status_t lidar_turn_on(lidar_t* lidar) {
	memset(lidar->rx_buffer, 0, 10);
	uint8_t request[] = {0xCD, 0x01, 0x03, 0x04};
	if(HAL_UART_Transmit(&lidar->huart, request, 4, 2000) != HAL_OK) {
		return LIDAR_UART_ERROR;
	}

	HAL_StatusTypeDef s = HAL_UART_Receive_DMA(&lidar->huart, lidar->rx_buffer, 4);
	if(s != HAL_OK) {
		return LIDAR_UART_ERROR;
	}

	lidar_error_code_t error = get_lidar_error(lidar);
	if(error == LIDAR_ERROR_NONE || error == LIDAR_ERROR_LASER_ALREADY_ON) {
		lidar->state = LIDAR_ON;
		return LIDAR_OK;
	}

	return LIDAR_UNKNOWN_ERROR;
}

lidar_status_t lidar_turn_off(lidar_t* lidar) {
	uint8_t request[] = {0xCD, 0x01, 0x04, 0x05};
	if(HAL_UART_Transmit(&lidar->huart, request, 4, 2000) != HAL_OK) {
		return LIDAR_UART_ERROR;
	}

	if(HAL_UART_Receive_DMA(&lidar->huart, lidar->rx_buffer, 4) != HAL_OK) {
		return LIDAR_UART_ERROR;
	}

	lidar_error_code_t error = get_lidar_error(lidar);
	if(error == LIDAR_ERROR_NONE) {
		lidar->state = LIDAR_OFF;
		return LIDAR_OK;
	}

	return LIDAR_UNKNOWN_ERROR;
}

lidar_status_t lidar_measure(lidar_t* lidar, int *out_distance) {
	uint8_t request[] = {0xCD, 0x01, 0x06, 0x07};
	if(HAL_UART_Transmit(&lidar->huart, request, 4, 2000) != HAL_OK) {
		return LIDAR_UART_ERROR;
	}

	HAL_StatusTypeDef s = HAL_UART_Receive_DMA(&lidar->huart, lidar->rx_buffer, 8);
	if(s != HAL_OK) {
		return LIDAR_UART_ERROR;
	}

	lidar_error_code_t error = get_lidar_error(lidar);

	switch(error) {
	case LIDAR_ERROR_CHECKSUM:
		return LIDAR_INVALID_CHECKSUM;
	case LIDAR_ERROR_OUT_OF_RANGE:
		return LIDAR_OUT_OF_RANGE;
	case LIDAR_ERROR_LASER_NOT_ON:
		return LIDAR_LASER_NOT_ON;
	case LIDAR_ERROR_LOW_SNR:
		return LIDAR_LOW_SNR;
	case LIDAR_ERROR_NONE: {
		  int lo = lidar->rx_buffer[3];
		  int hi = lidar->rx_buffer[4];
		  *out_distance = lo | (hi << 8);
		  return LIDAR_OK;
	}
	default:
		return LIDAR_UNKNOWN_ERROR;
	}
}

