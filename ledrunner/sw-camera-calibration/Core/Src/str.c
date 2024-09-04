/*
 * str.c
 *
 *  Created on: Mar 26, 2024
 *      Author: rapha
 */

#include "str.h"
#include <assert.h>

#define STR_BASIC_ASSERT(str) do { 		\
		assert(str); 				\
		assert(str->iE >= str->i0);	\
	} while(0)

uint16_t str_len(Str *str)
{
	STR_BASIC_ASSERT(str);
	uint16_t result = str->iE - str->i0;
	return result;
}

uint8_t str_at(Str *str, uint16_t index)
{
	STR_BASIC_ASSERT(str);
	assert(index < str->iE);
	assert(index >= str->i0);
	uint8_t result = str->s[index + str->i0];
	return result;
}
