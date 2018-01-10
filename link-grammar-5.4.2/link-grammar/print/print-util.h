/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/
#ifndef LG_PRINT_UTIL_H_
#define LG_PRINT_UTIL_H_

#if     __GNUC__ > 2 || (__GNUC__ == 2 && __GNUC_MINOR__ > 4)
#define GNUC_PRINTF( format_idx, arg_idx )    \
  __attribute__((__format__ (__printf__, format_idx, arg_idx)))
#else
#define GNUC_PRINTF( format_idx, arg_idx )
#endif

#include <stdlib.h>
#include <stdarg.h>

#include "dict-common/dict-defines.h" /* SUBSCRIPT_MARK, SUBSCRIPT_DOT */
#include "utilities.h"

int append_string(dyn_str *, const char *fmt, ...) GNUC_PRINTF(2,3);
int vappend_string(dyn_str *, const char *fmt, va_list args)
	GNUC_PRINTF(2,0);
size_t append_utf8_char(dyn_str *, const char * mbs);

static inline void patch_subscript_mark(char *s)
{
	s = strchr(s, SUBSCRIPT_MARK);
	if (NULL != s)
		*s = SUBSCRIPT_DOT;
}

static inline void patch_subscript_marks(char *s)
{
	while (NULL != (s = strchr(s, SUBSCRIPT_MARK)))
		*s = SUBSCRIPT_DOT;
}

static inline int display_width(int width, const char *s)
{
	return width + strlen(s) - utf8_strwidth(s);
}

#endif
