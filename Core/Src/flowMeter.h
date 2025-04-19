/*
 * flowMeter.h
 *
 *  Created on: Apr 7, 2025
 *      Author: beiss
 */

#ifndef SRC_FLOWMETER_H_
#define SRC_FLOWMETER_H_

extern uint16_t pulseCount;

//interrupt to count pulses
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	if (pin == GPIO_PIN_0){
		pulseCount++;
	}
}

#endif /* SRC_FLOWMETER_H_ */
