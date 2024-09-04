/*
 * tlc.h - TLC5926 utility
 *
 *  Created on: 20.06.2024
 *      Author: rapha
 *
 *
 */

#ifndef INC_TLC_H_
#define INC_TLC_H_

#define TLC_N_LEDS 					16
//#define TLC_CONTROLLING_ALL_ARR 	160-1 		// 170=minimum
#define TLC_CONTROLLING_ALL_ARR 	500-1 		// 450=min -> can't go faster than 5kHz - code bottled (?!) can't control over usb!
#define TLC_CONTROLLING_ALL_PSC 	0
#define TLC_CONTROLLING_DATA_ARR 	200-1		// 800 ok / 600ok
#define TLC_CONTROLLING_DATA_PSC 	0

#define TLC_PWM_MAX 				16

#include "stm32f3xx_hal.h"

#include <stdbool.h>
#include <stdint.h>


/******************************************************************************/
/* ENUMS **********************************************************************/
/******************************************************************************/

typedef enum {
	TLC_MODE_NONE,
	TLC_MODE_RIDER,
	TLC_MODE_PWM,
	TLC_MODE_DISPLAY,
	TLC_MODE_CLEAR,
	/* modes above */
	TLC_MODE__COUNT,
	/* misc stuff below */
} TlcModeList;

/******************************************************************************/
/* STRUCTS ********************************************************************/
/******************************************************************************/

typedef struct Tlc {
	struct {
		float self;
		float fps;
		uint32_t system;
	} freq;
	struct {
		uint32_t arr;
		uint32_t psc;
	} rider;
	struct {
		int *duty_cycle;
		int duty_max;
		bool *on;
	} pwm;
	struct {
		bool *data;
	} display;
	struct {
		bool *oe;
		bool *le;
		//TlcModeList *mode;
	} raw;
	TIM_TypeDef *tim;
	TIM_HandleTypeDef *htim;
	TlcModeList *mode;
	bool gpio_out;
	bool gpio_int;
	bool pause;
	bool running;
} Tlc;

/******************************************************************************/
/* FUNCTION PROTOTYPES ********************************************************/
/******************************************************************************/

void tlc_it_rider();
void tlc_it_pwm();
void tlc_it_display();
void tlc_it_none();

void tlc_mode_exit(Tlc *tlc, TlcModeList except_if);

void tlc_apply_changes(Tlc *tlc);

void tlc_run(Tlc *tlc, bool status);

void tlc_latch(Tlc *tlc, bool state);

void tlc_output(Tlc *tlc, bool state);

int tlc_set_arr(Tlc *tlc, uint32_t val);
int tlc_set_psc(Tlc *tlc, uint32_t val);
int tlc_set_fps(Tlc *tlc, float val);

int tlc_recalculate_fps(Tlc *tlc);

int tlc_init(Tlc *tlc, TIM_TypeDef *tim, TIM_HandleTypeDef *htim);

TlcModeList tlc_mode(Tlc *tlc, TlcModeList mode);

#endif /* INC_TLC_H_ */
