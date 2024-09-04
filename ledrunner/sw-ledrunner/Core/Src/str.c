/*
 * str.c
 *
 *  Created on: Jun 20, 2024
 *      Author: rapha
 */

#include "str.h"
#include <string.h>
#include <stdlib.h> // realloc

int str_reserve(Str *str, size_t cap)
{
	void *temp = realloc(str->s, cap);
	if(!temp) {
		return -1;
	}
	str->s = temp;
	str->cap = cap;
	return 0;
}

void str_cstr(Str *str, char cstr[], size_t len)
{
	if(!str) return;
	if(!cstr) return;
	size_t n = len < str_length(str) ? len : str_length(str);
	memcpy(cstr, str_iter_begin(str), n);
}

size_t str_length(Str *str)
{
	if(!str) return 0;
	size_t len = str->last - str->first;
	if(str->first > str->last) len = 0;
	return len;
}

char str_at(Str *str, size_t i)
{
	if(!str) return 0;
	char c = 0;
	if(str->first + i < str->last) {
		c = str->s[str->first + i];
	}
	return c;
}

void str_clear(Str *str)
{
	if(!str) return;
	str->first = 0;
	str->last = 0;
}

char *str_iter_begin(Str *str)
{
	if(!str) return 0;
	char *s = &str->s[str->first];
	return s;
}

int str_cmp_cstr_start(Str *str, char *cstr)
{
	if(!str) return -1;
	if(!cstr) return -1;
	size_t len = strlen(cstr);
	if(len > str_length(str)) return -1;
	int result = strncmp(str_iter_begin(str), cstr, len);
	return result;
}

int str_cmp_start(Str *str, Str *sub, bool finish_with_ws_or_end)
{
	if(!str) return -1;
	if(!sub) return -1;
	if(str_length(sub) > str_length(str)) return -1;
	int result = strncmp(str_iter_begin(str), str_iter_begin(sub), str_length(sub));
	if(finish_with_ws_or_end && str_length(sub) < str_length(str)) {
		char finish = str_at(str, str_length(sub));
		if(!isspace(finish)) result = -1;
	}
	return result;
}

int str_app_c(Str *str, char c)
{
	if(!str) return -1;
	if(str->last + 1 > str->cap) return -1;
	str->s[str->last++] = c;
	return 0;
}

int str_app_cstr(Str *str, char *cstr, size_t len)
{
	if(!str) return -1;
	if(!cstr) return -1;
	if(str->last + len > str->cap) return -1;
	memcpy(&str->s[str->last], cstr, len);
	str->last += len;
	return 0;
}

size_t str_find_c(Str *str, char c)
{
	if(!str) return 0;
	size_t len = str_length(str);
	for(size_t i = 0; i < len; ++i) {
		char ci = str_at(str, i);
		if(ci == c) return i;
	}
	return len;
}

size_t str_find_nws(Str *str)
{
	if(!str) return 0;
	size_t len = str_length(str);
	for(size_t i = 0; i < len; ++i) {
		char ci = str_at(str, i);
		if(!isspace(ci)) return i;
	}
	return len;
}

size_t str_find_ws(Str *str)
{
	if(!str) return 0;
	size_t len = str_length(str);
	for(size_t i = 0; i < len; ++i) {
		char ci = str_at(str, i);
		if(isspace(ci)) return i;
	}
	return len;
}

size_t str_rfind_c(Str *str, char c)
{
	if(!str) return 0;
	size_t len = str_length(str);
	for(size_t i = len; i > 0; --i) {
		char ci = str_at(str, i - 1);
		if(ci == c) return i - 1;
	}
	return len;
}

