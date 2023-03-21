/*
 * laser.h
 *
 *  Created on: Feb 11, 2023
 *      Author: emilnorden
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "stm32f7xx_hal.h"
#include <memory>

enum class LidarState {
	Idle,
	Measuring,
};


class Lidar {
public:
	bool start_measure();

	static std::shared_ptr<Lidar> create(UART_HandleTypeDef *huart, IRQn_Type uart_irq);
private:
	explicit Lidar(UART_HandleTypeDef *huart);

	UART_HandleTypeDef *m_huart;
	LidarState m_state;
	uint8_t rx_header_buffer[4];
};

typedef enum {
	FALSE,
	TRUE
} success_t;

success_t lidar_init(UART_HandleTypeDef *huart);
success_t lidar_turn_on(void);
success_t lidar_turn_off(void);
success_t lidar_begin_measure(void);
success_t lidar_measure(void);
int lidar_get_measurement(void);

#endif /* INC_LIDAR_H_ */
