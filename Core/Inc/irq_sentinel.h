/*
 * irq_sentinel.h
 *
 *  Created on: Mar 13, 2023
 *      Author: emilnorden
 */

#ifndef INC_IRQ_SENTINEL_H_
#define INC_IRQ_SENTINEL_H_

#include "stm32f7xx_hal.h"

class ScopedDisabledIRQ {
public:
	explicit ScopedDisabledIRQ(IRQn_Type irq)
		: m_irq(irq) {
		HAL_NVIC_DisableIRQ(irq);
	}

	~ScopedDisabledIRQ() {
		HAL_NVIC_EnableIRQ(irq);
	}
private:
	IRQn_Type m_irq;
};


#endif /* INC_IRQ_SENTINEL_H_ */
