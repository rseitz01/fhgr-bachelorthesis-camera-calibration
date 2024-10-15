/*
 * tlc.c - TLC5926 utility
 *
 *  Created on: 20.06.2024
 *      Author: rapha
 */

#include <string.h>

#include "main.h" // gpio

#include "tlc.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_it.h"
#include "timer_estimator.h"

extern bool led_data[TLC_N_LEDS];
extern int led_pwm;
extern bool led_pwm_active;

extern bool led_oe;
extern bool led_le;
extern TlcModeList led_mode;

extern void (*led_function)(void);

int tlc_init(Tlc *tlc, TIM_TypeDef *tim, TIM_HandleTypeDef *htim)
{
	memset(tlc, 0, sizeof(*tlc));
	tlc->freq.system = HAL_RCC_GetSysClockFreq();
	tlc->tim = tim;
	tlc->htim = htim;
	tlc->pwm.duty_max = TLC_PWM_MAX;
	tlc->pwm.duty_cycle = &led_pwm;
	tlc->pwm.on = &led_pwm_active;
	tlc->display.data = led_data;
	//tlc->raw.le = &led_le;
	//tlc->raw.oe = &led_oe;
	tlc->mode = &led_mode;
	//tlc->raw.function = tlc_it_none();
	tlc_mode(tlc, TLC_MODE_CLEAR);
	return 0;
}

void tlc_pause(Tlc *tlc, bool status)
{
	tlc->pause = status;
	if(tlc->running) {
		if(tlc->pause) {
			HAL_TIM_Base_Stop_IT(tlc->htim);
			//HAL_TIM_PWM_Stop(tlc->htim, TIM_CHANNEL_1);
		} else {
			//HAL_TIM_PWM_Start(tlc->htim, TIM_CHANNEL_1);
			HAL_TIM_Base_Start_IT(tlc->htim);
		}
	}
	/*if(!tlc->pause) {
		tlc_mode_enter(tlc, tlc->mode);
	}*/
}

void tlc_run(Tlc *tlc, bool status)
{
	tlc->running = status;
	if(!tlc->running) {
		HAL_TIM_PWM_Stop(tlc->htim, TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(tlc->htim);
	} else {
		HAL_TIM_Base_Start_IT(tlc->htim);
		HAL_TIM_PWM_Start(tlc->htim, TIM_CHANNEL_1);
	}
}

int tlc_set_arr(Tlc *tlc, uint32_t val)
{
	if(!tlc) return -1;
	//tlc_pause(tlc, true);
	tlc->rider.arr = val;
	//tlc->tim->CCR1 = TIM2->ARR / 2;
	tlc_recalculate_fps(tlc);
	//tlc_pause(tlc, false);
	tlc_apply_changes(tlc);
	return 0;
}

int tlc_set_psc(Tlc *tlc, uint32_t val)
{
	if(!tlc) return -1;
	//tlc_pause(tlc, true);
	tlc->rider.psc = val;
	tlc_recalculate_fps(tlc);
	//tlc_pause(tlc, false);
	tlc_apply_changes(tlc);
	return 0;
}

int tlc_recalculate_fps(Tlc *tlc)
{
	if(!tlc) return -1;
	if(!tlc->tim->ARR) return -1;
	TimerEstimator te = {
			.arr = tlc->rider.arr,
			.psc = tlc->rider.psc,
	};
    tlc->freq.fps = timer_estimate_freq_toggle(te, tlc->freq.system) / (float)(2 * TLC_N_LEDS); //(float)tlc->freq.system / (float)(2 * TLC_N_LEDS) / (tlc->rider.psc + 1) / tlc->rider.arr;
	return 0;
}

int tlc_set_fps(Tlc *tlc, float val)
{
	//tlc_pause(tlc, true);
    TimerEstimator est = {0};
    tlc->freq.self = 2.0f * (float)TLC_N_LEDS * val;
    uint32_t iter = timer_estimate(&est, tlc->freq.system, tlc->freq.self);
	tlc->rider.psc = est.psc;
	tlc->rider.arr = est.arr;
	//tlc->tim->CCR1 = est.arr / 2;
	tlc_recalculate_fps(tlc);
	tlc_apply_changes(tlc);
	//tlc_pause(tlc, false);
	return 0;
}

void tlc_apply_arr_psc(Tlc *tlc, uint32_t psc, uint32_t arr)
{
	if(!tlc) return;
	tlc->tim->ARR = arr;
	tlc->tim->PSC = psc;
	//tlc->tim->CCR1 = TIM2->ARR / 2;
	tlc->tim->CNT = 0;
}

void tlc_mode_exit(Tlc *tlc, TlcModeList except_if)
{
	if(!tlc->mode) return;
	if(*tlc->mode == except_if) return;

	HAL_GPIO_WritePin(TLC5926_OE_GPIO_Port, TLC5926_OE_Pin, GPIO_PIN_SET); /* high OE = off LEDs */
	HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_RESET); /* low LE = don't transfer to latch */

#if 0
	switch(*tlc->mode) {
	case TLC_MODE_NONE: {
	} break;
	case TLC_MODE_PWM: {
	} break;
	case TLC_MODE_RIDER: {
	} break;
	case TLC_MODE_DISPLAY: {
	} break;
	case TLC_MODE__COUNT: {
	} break;
	}
#endif

	*tlc->pwm.on = false;
	*tlc->pwm.duty_cycle = -1;
	*tlc->mode = TLC_MODE_NONE;
	tlc_apply_arr_psc(tlc, 0, 0);
}

void tlc_apply_changes(Tlc *tlc)
{
	if(!tlc->running || tlc->pause) return;
	if(!tlc->mode) return;
	switch(*tlc->mode) {
	case TLC_MODE_RIDER: {
		//tlc_mode_enter(tlc, TLC_MODE_RIDER);
		tlc_pause(tlc, true);
		tlc_apply_arr_psc(tlc, tlc->rider.psc, tlc->rider.arr);
		tlc_pause(tlc, false);
	} break;
	default: {} break;
	}
}

void tlc_mode_enter(Tlc *tlc, TlcModeList id)
{
	if(*tlc->mode != TLC_MODE_NONE/* && tlc->mode != id*/) return;
	//HAL_Delay(100);
	switch(id) {
	case TLC_MODE_NONE: {
	} break;
	case TLC_MODE_PWM: {
		if(*tlc->pwm.duty_cycle < 0) {
			*tlc->pwm.duty_cycle = 0;	// 0..10 // TODO:init or not??
		}
		*tlc->pwm.on = true;
		tlc_apply_arr_psc(tlc, TLC_CONTROLLING_ALL_PSC, TLC_CONTROLLING_ALL_ARR);
		//*tlc->raw.le = false;
		//*tlc->raw.oe = false;
		//HAL_Delay(100);
		HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_SET); /* high LE = transfer to latch */
		//HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_RESET); /* low LE = don't transfer to latch */
		HAL_GPIO_WritePin(TLC5926_OE_GPIO_Port, TLC5926_OE_Pin, GPIO_PIN_RESET); /* low OE = drive LEDs */
	} break;
	case TLC_MODE_RIDER: {
		*tlc->pwm.duty_cycle = -1;
		*tlc->pwm.on = false;
		tlc_apply_arr_psc(tlc, tlc->rider.psc, tlc->rider.arr);
		////*tlc->raw.le = true;
		//HAL_Delay(100);
		HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_SET); /* high LE = transfer to latch */
		HAL_GPIO_WritePin(TLC5926_OE_GPIO_Port, TLC5926_OE_Pin, GPIO_PIN_RESET); /* low OE = drive LEDs */
	} break;
	case TLC_MODE_DISPLAY: {
		*tlc->pwm.duty_cycle = 0;	// >=0
		*tlc->pwm.on = false;
		//memset(tlc->display.data, 0, sizeof(*tlc->display.data) * TLC_N_LEDS); // TODO:init or not??
		tlc_apply_arr_psc(tlc, TLC_CONTROLLING_DATA_PSC, TLC_CONTROLLING_DATA_ARR);
		//*tlc->raw.le = false;
		//*tlc->raw.oe = false;
		//HAL_Delay(100);
		HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_RESET); /* low LE = don't transfer to latch */
		HAL_GPIO_WritePin(TLC5926_OE_GPIO_Port, TLC5926_OE_Pin, GPIO_PIN_RESET); /* low OE = drive LEDs */
	} break;
	case TLC_MODE_CLEAR: {
		tlc_apply_arr_psc(tlc, TLC_CONTROLLING_ALL_PSC, TLC_CONTROLLING_ALL_ARR);
	} break;
	case TLC_MODE__COUNT: {} break;
	}
	if(id < TLC_MODE__COUNT) {
		*tlc->mode = id;
	}
}

TlcModeList tlc_mode(Tlc *tlc, TlcModeList mode)
{
	if(!tlc->mode) return TLC_MODE_NONE;
	if(*tlc->mode != mode) {
		tlc_pause(tlc, true);
		tlc_mode_exit(tlc, TLC_MODE_NONE);
		/* make sure the LEDs are cleared */
		bool was_running = tlc->running;
		tlc_mode_enter(tlc, TLC_MODE_CLEAR);
		tlc_pause(tlc, false);
		tlc_run(tlc, true);
		while(*tlc->mode != TLC_MODE_NONE);
		tlc_pause(tlc, true);
		tlc_run(tlc, was_running);
		/* finally, enter mode */
		tlc_mode_enter(tlc, mode);
		tlc_pause(tlc, false);
	} else {
		//tlc_apply_changes(tlc);
	}
	return *tlc->mode;
}

