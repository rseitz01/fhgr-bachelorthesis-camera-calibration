/*
 * str.h
 *
 *  Created on: Jun 20, 2024
 *      Author: rapha
 */

#ifndef INC_STR_H_
#define INC_STR_H_

#include <stddef.h>
#include <stdbool.h>

#define CSTR(x) 	x, strlen(x)

#define STR(x) 		(Str){.s = x, .last = sizeof(x)/sizeof(*x) - 1}

typedef struct Str {
	size_t cap;
	size_t first;
	size_t last;
	char *s;
} Str;

#define STR_INFOPRINT(x) 	do { \
		char buf[256] = {0};	\
		snprintf(buf, 256, "\r\n[%u..%u]:%.*s\r\n", (x).first, (x).last, (int)str_length(&(x)), str_iter_begin(&(x))); \
		cli_raw_send(buf); \
	} while(0)

size_t str_length(Str *str);
char str_at(Str *str, size_t i);
void str_clear(Str *str);

char *str_iter_begin(Str *str);

int str_reserve(Str *str, size_t cap);
void str_cstr(Str *str, char cstr[], size_t len);

int str_cmp_cstr_start(Str *str, char *cstr);
int str_cmp_start(Str *str, Str *sub, bool finish_with_ws_or_end);

int str_app_c(Str *str, char c);
int str_app_cstr(Str *str, char *cstr, size_t len);

size_t str_find_c(Str *str, char c);
size_t str_find_nws(Str *str);
size_t str_find_ws(Str *str);
size_t str_rfind_c(Str *str, char c);


#endif /* INC_STR_H_ */
