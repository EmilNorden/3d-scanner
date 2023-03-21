#include "lidar.h"
#include "irq_sentinel.h"
#include <stdexcept>
#include <unordered_map>

typedef enum {
	LIDAR_STATE_OFF,
	LIDAR_STATE_ON,
	LIDAR_STATE_ENTERING_MEASURING,
	LIDAR_STATE_MEASURING,
	LIDAR_STATE_FAULTED,
	LIDAR_STATE_PENDING_TURN_OFF,
} lidar_state_t;

typedef enum {
	REQUEST_IDLE, REQUEST_COMPLETE, REQUEST_ERROR, REQUEST_TIMEOUT
} lidar_request_state_t;

static UART_HandleTypeDef *lidar_huart;
static uint8_t rx_header_buffer[4];
static uint8_t rx_payload_buffer[16];
static uint8_t rx_measure_buffer[20];

static lidar_state_t current_state;
static lidar_request_state_t current_request_state;
static int last_error;
static volatile int latest_measurement = -1;

success_t send_uart_request_blocking(const uint8_t *data, uint16_t size,
		uint16_t recv_size);
success_t start_dma_receive(int size);
uint8_t get_response_byte(int index, int offset);
success_t validate_measurement_response(int offset);
void lidar_HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void lidar_HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
static success_t start_receive_header(void);
static success_t wait_for_success(uint32_t timeout);
static success_t header_is_ok(void);

#define MEASURE_RESPONSE_SIZE	16
std::unordered_map<UART_HandleTypeDef*, std::shared_ptr<Lidar>> active_lidars;

static std::shared_ptr<Lidar> Lidar::create(UART_HandleTypeDef *huart, IRQn_Type uart_irq) {
	ScopedDisabledIRQ sentinel(uart_irq);
	auto active_lidar = active_lidars.find(huart);
	if(active_lidar == active_lidars.end()) {
		auto result = std::make_shared<Lidar>(new Lidar(huart));

		active_lidars.emplace(huart, result);

		return result;
	}

	return active_lidar->second;
}

Lidar::Lidar(UART_HandleTypeDef *huart)
: m_huart(huart) {
	if(HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, lidar_HAL_UART_RxCpltCallback) != HAL_OK) {
		throw std::runtime_error("Unable to register UART Complete callback");
	}

	if(HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, lidar_HAL_UART_ErrorCallback) != HAL_OK) {
		throw std::runtime_error("Unable to register UART error callback");
	}
}

bool Lidar::start_measure() {
	const uint8_t request[] = { 0xCD, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x04 };


	// Initial response will be FA 00 01 01
	if (HAL_UART_Receive_IT(lidar_huart, rx_header_buffer, 4) != HAL_OK) {
		return false;
	}

	if (HAL_UART_Transmit(lidar_huart, request, 12, 2000) != HAL_OK) {
		return false;
	}

	return true;
}


success_t lidar_init(UART_HandleTypeDef *huart) {
	lidar_huart = huart;
	current_state = LIDAR_STATE_OFF;

	if(HAL_UART_RegisterCallback(lidar_huart, HAL_UART_RX_COMPLETE_CB_ID, lidar_HAL_UART_RxCpltCallback) != HAL_OK) {
		return FALSE;
	}

	if(HAL_UART_RegisterCallback(lidar_huart, HAL_UART_ERROR_CB_ID, lidar_HAL_UART_ErrorCallback) != HAL_OK) {
		return FALSE;
	}

	return TRUE;
}

success_t lidar_turn_on(void) {
	const uint8_t request[] = { 0xCD, 0x01, 0x03, 0x04 };

	success_t res = send_uart_request_blocking(request, 4, 4);
	if (res) {
		current_state = LIDAR_STATE_ON;
	}

	return res;
}

success_t lidar_turn_off(void) {
	const uint8_t request[] = { 0xCD, 0x01, 0x04, 0x05 };

	if(!start_receive_header()) {
		return FALSE;
	}

	current_request_state = REQUEST_IDLE;
	if (HAL_UART_Transmit(lidar_huart, request, 12, 2000) != HAL_OK) {
			return FALSE;
	}

	success_t result = wait_for_success(2000);
	if (result) {
		current_state = LIDAR_STATE_OFF;
	}

	return result;
}

success_t send_uart_request_blocking_XXX(const uint8_t *data, uint16_t size) {

	while (current_request_state != REQUEST_COMPLETE
			&& current_request_state != REQUEST_ERROR) {
	}
	success_t result = current_request_state == REQUEST_COMPLETE ? TRUE : FALSE;
	current_request_state = REQUEST_IDLE;

	return result;
}

success_t lidar_begin_measure(void) {
	current_state = LIDAR_STATE_ENTERING_MEASURING;
	// I'm only supporting mode C0 at the moment.
	// The other modes have a weird "TBD" note on the measurement frequency in the documentation.
	const uint8_t request[] = { 0xCD, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x04 };

	// Initial response will be FA 00 01 01
	if (HAL_UART_Receive_IT(lidar_huart, rx_header_buffer, 4) != HAL_OK) {
		return FALSE;
	}

	if (HAL_UART_Transmit(lidar_huart, request, 12, 2000) != HAL_OK) {
		return FALSE;
	}

	return TRUE;
}

int lidar_get_measurement(void) {
	HAL_NVIC_DisableIRQ(USART2_IRQn);

	int result = latest_measurement;

	HAL_NVIC_EnableIRQ(USART2_IRQn);

	return result;
}

success_t send_uart_request_blocking(const uint8_t *data, uint16_t size,
		uint16_t recv_size) {
	current_request_state = REQUEST_IDLE;

	/*if(start_dma_receive(recv_size) == FALSE) {
	 return FALSE;
	 }*/

	while (current_request_state != REQUEST_COMPLETE
			&& current_request_state != REQUEST_ERROR) {
	}
	success_t result = current_request_state == REQUEST_COMPLETE ? TRUE : FALSE;
	current_request_state = REQUEST_IDLE;

	return result;
}

void lidar_HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//HAL_UART_Receive_IT(&huart2, Rx_data, 4);
	if(current_state == LIDAR_STATE_ENTERING_MEASURING) {
		// TODO Validate that rxbuffer is FA 00 01 01. If not we should go to some error state
		current_state = LIDAR_STATE_MEASURING;
		HAL_StatusTypeDef res = HAL_UART_Receive_IT(lidar_huart, rx_payload_buffer, 16);
		int foo = 34;
	}
	else if(current_state == LIDAR_STATE_MEASURING) {

		int foo = 34;
		UNUSED(foo);
		if(validate_measurement_response(0)) {
			latest_measurement = rx_payload_buffer[3] | (rx_payload_buffer[4] << 8);
		}

		HAL_StatusTypeDef res = HAL_UART_Receive_IT(lidar_huart, rx_payload_buffer, 16);
	}
	else if(current_state == LIDAR_STATE_PENDING_TURN_OFF) {
		if(header_is_ok()) {
			current_request_state = REQUEST_COMPLETE;
		}
		else {
			current_request_state = REQUEST_ERROR;
			last_error = 0;
		}
	}
}

void lidar_HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

	current_state = LIDAR_STATE_PENDING_TURN_OFF;
	lidar_turn_off();

}

uint8_t get_response_byte(int index, int offset) {
	int final_index = (offset + index) % MEASURE_RESPONSE_SIZE;
	return rx_payload_buffer[final_index];
}

success_t validate_measurement_response(int offset) {
	// Header
	if (get_response_byte(0, offset) != 0xFA) {
		return FALSE;
	}

	// Length (always 13/0x0D)
	if (get_response_byte(1, offset) != 0x00
			|| get_response_byte(2, offset) != 0x0D) {
		return FALSE;
	}

	uint8_t checksum = 0;
	for (int i = 1; i < 15; ++i) {
		checksum += get_response_byte(i, offset);
	}

	if (get_response_byte(15, offset) != checksum) {
		return FALSE;
	}

	return TRUE;
}

success_t lidar_measure(void) {
	//static uint8_t rx_measure_buffer[20];
	current_state = LIDAR_STATE_ENTERING_MEASURING;
	// I'm only supporting mode C0 at the moment.
	// The other modes have a weird "TBD" note on the measurement frequency in the documentation.
	const uint8_t request[] = { 0xCD, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x04 };

	// Initial response will be FA 00 01 01
	if (HAL_UART_Receive_IT(lidar_huart, rx_measure_buffer, 20) != HAL_OK) {
		return FALSE;
	}

	if (HAL_UART_Transmit(lidar_huart, request, 12, 2000) != HAL_OK) {
		return FALSE;
	}

	return TRUE;
}

static success_t start_receive_header(void) {
	if(HAL_UART_Receive_IT(lidar_huart, rx_header_buffer, 4) == HAL_OK) {
		return TRUE;
	}

	return FALSE;
}

static success_t wait_for_success(uint32_t timeout) {
	// Timeout code taken from HAL_Delay
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = timeout;

	/* Add a freq to guarantee minimum wait */
	if (timeout < HAL_MAX_DELAY)
	{
		timeout += (uint32_t)(uwTickFreq);
	}

	while (current_request_state != REQUEST_COMPLETE
			&& current_request_state != REQUEST_ERROR &&
			(HAL_GetTick() - tickstart) < timeout) {
	}

	if(current_request_state == REQUEST_COMPLETE) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}

static success_t header_is_ok(void) {
	if(rx_header_buffer[0] != 0xFA ||
			rx_header_buffer[1] != 0x00 ||
			rx_header_buffer[2] != 0x01 ||
			rx_header_buffer[3] != 0x01) {
		return FALSE;
	}

	return TRUE;
}

