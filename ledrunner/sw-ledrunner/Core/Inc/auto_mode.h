/*
 * auto_mode.h
 *
 *  Created on: Jul 4, 2024
 *      Author: rapha
 */

#ifndef SRC_AUTO_MODE_H_
#define SRC_AUTO_MODE_H_

#define AUTO_MODE_DEFAULT_FPS_MULTIPLE 		10

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	AUTO_MODE_NONE,
	AUTO_MODE_ALL_OFF,
	AUTO_MODE_ALL_LIT,
	AUTO_MODE_DIRECTION,
	AUTO_MODE_SYS_FREQ,
	AUTO_MODE_TIMER_ARR,
	AUTO_MODE_TIMER_PSC,
	AUTO_MODE_PWM_COUNT,
	AUTO_MODE_PWM,
	AUTO_MODE_PWM_DONE,
	AUTO_MODE_RIDER,
} AutoModeList;

typedef struct AutoMode {
	AutoModeList step_next;
	AutoModeList step_prev;
	bool active;
	uint32_t tick_prev;
	uint32_t ms_wait_between_steps;
	uint16_t pwm_duty_cycle;
} AutoMode;

typedef struct Tlc Tlc;

void auto_mode_activate(AutoMode *am, float fps);
void auto_mode_deactivate(AutoMode *am);
void auto_mode_process(AutoMode *am, Tlc *tlc);

#endif /* SRC_AUTO_MODE_H_ */
