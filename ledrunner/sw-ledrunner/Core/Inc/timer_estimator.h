/*
 * timer_estimator.h
 *
 *  Created on: Jun 20, 2024
 *      Author: rapha
 */

#ifndef INC_TIMER_ESTIMATOR_H_
#define INC_TIMER_ESTIMATOR_H_

#include <stdint.h>

typedef struct {
	uint32_t psc;
	uint32_t arr;
} TimerEstimator;

float timer_estimate_freq_toggle(const TimerEstimator timer, uint32_t base);
uint32_t timer_estimate(TimerEstimator *est, uint32_t base_freq, float goal_freq);

#endif /* INC_TIMER_ESTIMATOR_H_ */
