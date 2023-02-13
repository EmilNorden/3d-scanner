#include "lidar.h"

typedef enum {
	LIDAR_NONE,
	LIDAR_IN_PROGRESS,
	LIDAR_COMPLETE,
	LIDAR_IDLE
} lidar_request_state_t;

typedef enum {
	REQ_STAT_OK,
	REQ_STAT_BUSY,
	REQ_STAT_UART_ERROR
} uart_request_status_t;

static UART_HandleTypeDef *lidar_huart;
static uint8_t rxbuffer[8];
static lidar_request_state_t current_request_state;

uart_request_status_t send_uart_request(const uint8_t* data, uint16_t size);

void lidar_init(UART_HandleTypeDef *huart)
{
	lidar_huart = huart;
	current_request_state = LIDAR_NONE;
	// TODO Should handle return value!!
	HAL_UART_RegisterRxEventCallback(lidar_huart, HAL_UARTEx_RxEventCallback);
	//HAL_UARTEx_ReceiveToIdle_DMA(lidar_huart, rxbuffer, 8);
}

success_t lidar_turn_on(void) {
	const uint8_t request[] = {0xCD, 0x01, 0x03, 0x04};
	uart_request_status_t res = send_uart_request(request, 4);

	if(res == REQ_STAT_OK) {
		return TRUE;
	}
	return FALSE;
}

success_t lidar_turn_off(void) {
	const uint8_t request[] = {0xCD, 0x01, 0x04, 0x05};
	uart_request_status_t res = send_uart_request(request, 4);

	if(res == REQ_STAT_OK) {
		return TRUE;
	}
	return FALSE;
}

success_t lidar_measure(int *out_distance) {
	const uint8_t request[] = {0xCD, 0x01, 0x06, 0x07};
	uart_request_status_t res = send_uart_request(request, 4);

	if(res == REQ_STAT_OK) {
		int lo = rxbuffer[3];
		int hi = rxbuffer[4];
		*out_distance = lo | (hi << 8);

		return TRUE;
	}
	return FALSE;
}


uart_request_status_t send_uart_request(const uint8_t* data, uint16_t size) {
	if(current_request_state == LIDAR_IN_PROGRESS) {
		return REQ_STAT_BUSY;
	}

	HAL_StatusTypeDef res;
	res = HAL_UARTEx_ReceiveToIdle_DMA(lidar_huart, rxbuffer, 8);
	if(res != HAL_OK) {
		return REQ_STAT_UART_ERROR;
	}

	res = HAL_UART_Transmit(lidar_huart, data, size, 2000);
	if(res != HAL_OK) {
		return REQ_STAT_UART_ERROR;
	}

	while(current_request_state != LIDAR_COMPLETE && current_request_state != LIDAR_IDLE) {}
	current_request_state = LIDAR_NONE;

	return REQ_STAT_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Pos) {
	  /* Prevent unused argument(s) compilation warning */
	  UNUSED(Pos);
/*  *           HAL_UART_RXEVENT_TC                 = 0x00U,
  *           HAL_UART_RXEVENT_HT                 = 0x01U,
  *           HAL_UART_RXEVENT_IDLE               = 0x02U,
  */
	  switch(huart->RxEventType) {
	  case HAL_UART_RXEVENT_TC:
		  current_request_state = LIDAR_COMPLETE;
		  break;
	  case HAL_UART_RXEVENT_HT:
		  // Ignore this event
		  break;
	  case HAL_UART_RXEVENT_IDLE:
		  current_request_state = LIDAR_IDLE;
		  break;
	  }

}
