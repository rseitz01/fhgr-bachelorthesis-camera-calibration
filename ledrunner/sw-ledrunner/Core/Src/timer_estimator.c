/*
 * timer_estimator.c
 *
 *  Created on: Jun 20, 2024
 *      Author: rapha
 */

#include <math.h>
#include "timer_estimator.h"

#define RELOAD_MIN 	117

float timer_estimate_freq_toggle(const TimerEstimator timer, uint32_t base)
{
    if(!timer.arr) return 0;
    float freq = (float)base/(float)(timer.psc + 1)/(float)(timer.arr + 1);
    return freq;
}

uint32_t timer_estimate(TimerEstimator *est, uint32_t base_freq, float goal_freq)
{
	if(!est) return 0;

	TimerEstimator best = {0};
	uint32_t iteration = 0;

	if(!goal_freq) {
		est->arr = 0;
		return 0; // TODO: or return -1??
	}
	if(goal_freq > base_freq) {
		return 0;
	}

	TimerEstimator propose = best;
	float f_goal = (float)goal_freq;
	float err = INFINITY;
	float base = base_freq / goal_freq;
	float max = 2 * sqrt(UINT16_MAX);

	for (uint16_t i = 1; i < max; ++i) {
		float reload = base / (float)i - 1;
		if(reload < RELOAD_MIN) continue;
		propose.arr = roundf(reload);
		for (uint16_t j = 1; j < max; ++j) {
			++iteration;
			float prescaler = reload / (float)j - 1;
			propose.psc = roundf(prescaler);
			float f_result = timer_estimate_freq_toggle(propose, base_freq);
			float f_err = (f_goal - f_result) * (f_goal - f_result);
			if(f_err < err) {
				err = f_err;
				best = propose;
				if(!err) goto exit;
			}
		}
	}
exit:
	*est = best;
	return iteration;
}


