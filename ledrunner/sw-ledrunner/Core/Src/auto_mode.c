/*
 * auto_mode.c
 *
 *  Created on: Jul 4, 2024
 *      Author: rapha
 */

#include "auto_mode.h"
#include "tlc.h"

void auto_mode_activate(AutoMode *am, float fps)
{
	if(!am) return;
	am->active = true;
	am->ms_wait_between_steps = (AUTO_MODE_DEFAULT_FPS_MULTIPLE * 1000.0f / fps + 0.5f);
	am->step_next = AUTO_MODE_NONE;
	am->step_prev = AUTO_MODE_NONE;
	am->pwm_duty_cycle = 0;
}

void auto_mode_deactivate(AutoMode *am)
{
	if(!am) return;
	am->active = false;
	//tlc_run(tlc, false); // TODO this is not good
}

void auto_mode_process(AutoMode *am, Tlc *tlc)
{
	if(!am) return;
	if(!am->active) return;
	uint32_t tick_next = HAL_GetTick();
	if(tick_next - am->tick_prev >= am->ms_wait_between_steps) {
		if(am->step_next == AUTO_MODE_PWM) {
			++am->pwm_duty_cycle;
			if(am->pwm_duty_cycle > tlc->pwm.duty_max) {
				++am->step_next;
			}
		} else if(am->step_next < AUTO_MODE_RIDER) {
			++am->step_next;
		}
		am->tick_prev = tick_next;
		am->step_prev = am->step_next;
	} else {
		return;
	}
	//if(am->step_next == am->step_prev) return;
	//tlc_mode_exit(tlc, TLC_MODE_NONE);
	switch(am->step_next) {
	case AUTO_MODE_NONE: {} break;
	case AUTO_MODE_ALL_OFF: {
		tlc_run(tlc, true);
		tlc_mode(tlc, TLC_MODE_CLEAR);
	} break;
	case AUTO_MODE_ALL_LIT: {
		*tlc->pwm.duty_cycle = tlc->pwm.duty_max;
		tlc_mode(tlc, TLC_MODE_PWM);
	} break;
	case AUTO_MODE_PWM_COUNT: {
		uint16_t num = TLC_PWM_MAX;
		for(size_t i = 0; i < TLC_N_LEDS; ++i) tlc->display.data[i] = ((num >> i) & 1);
		tlc_mode(tlc, TLC_MODE_DISPLAY);
	} break;
	case AUTO_MODE_PWM: {
		*tlc->pwm.duty_cycle = am->pwm_duty_cycle;
		tlc_mode(tlc, TLC_MODE_PWM);
	} break;
	case AUTO_MODE_PWM_DONE: {
		tlc_mode(tlc, TLC_MODE_NONE);
	} break;
	case AUTO_MODE_DIRECTION: {
		uint16_t num = 0x842F;
		for(size_t i = 0; i < TLC_N_LEDS; ++i) tlc->display.data[i] = ((num >> i) & 1);
		tlc_mode(tlc, TLC_MODE_DISPLAY);
	} break;
	case AUTO_MODE_SYS_FREQ: {
		uint16_t num = tlc->freq.system / 1000000;
		for(size_t i = 0; i < TLC_N_LEDS; ++i) tlc->display.data[i] = ((num >> i) & 1);
		tlc_mode(tlc, TLC_MODE_DISPLAY);
	} break;
	case AUTO_MODE_TIMER_ARR: {
		uint16_t num = tlc->rider.arr;
		for(size_t i = 0; i < TLC_N_LEDS; ++i) tlc->display.data[i] = ((num >> i) & 1);
		tlc_mode(tlc, TLC_MODE_DISPLAY);
	} break;
	case AUTO_MODE_TIMER_PSC: {
		uint16_t num = tlc->rider.psc;
		for(size_t i = 0; i < TLC_N_LEDS; ++i) tlc->display.data[i] = ((num >> i) & 1);
		tlc_mode(tlc, TLC_MODE_DISPLAY);
	} break;
	case AUTO_MODE_RIDER: {
		tlc_mode(tlc, TLC_MODE_RIDER);
	} break;
	default: {} break;
	}
}
