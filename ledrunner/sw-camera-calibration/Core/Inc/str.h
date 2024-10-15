/*
 * str.h
 *
 *  Created on: Mar 26, 2024
 *      Author: rapha
 */

#ifndef SRC_STR_H_
#define SRC_STR_H_

#include <stdint.h>

typedef struct Str {
	uint16_t i0;
	uint16_t iE;
	uint16_t cap;
	uint8_t s[];
} Str;


uint16_t str_len(Str *str);
uint8_t str_at(Str *str, uint16_t index);

#endif /* SRC_STR_H_ */
