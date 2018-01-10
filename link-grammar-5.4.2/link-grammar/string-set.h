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
#ifndef _STRING_SET_H_
#define _STRING_SET_H_

#include <string.h>
#include <stddef.h>

#include "api-types.h"
#include "lg_assert.h"

struct String_set_s
{
   size_t size;       /* the current size of the table */
   size_t count;      /* number of things currently in the table */
   char ** table;     /* the table itself */
};

String_set * string_set_create(void);
const char * string_set_add(const char * source_string, String_set * ss);
const char * string_set_lookup(const char * source_string, String_set * ss);
void         string_set_delete(String_set *ss);

/**
 * Compare 2 strings, assuming they are in the same string-set.
 * Two string-set strings are equal if and only if their pointers are equal.
 * Return true if they are equal, else false.
 * In debug mode, also "validate" that the strings are indeed from the
 * same string-set, and that their comparison is as expected.
 * This validation may be false positive but the probability is very low.
 */
static inline bool string_set_cmp(const char *s1, const char *s2)
{
#ifdef DEBUG
	size_t p1 = ((strlen(s1)+1)&~(sizeof(String_set *)-1))+sizeof(String_set *);
	size_t p2 = ((strlen(s2)+1)&~(sizeof(String_set *)-1))+sizeof(String_set *);

	assert(*(String_set **)&s1[p1] == *(String_set **)&s2[p2],
	       "Strings '%s' and '%s' are not from the same string_set", s1, s2);
	assert((s1 != s2) == !!strcmp(s1, s2),
	       "Bogus string-set string comparison ('%s' and '%s')", s1, s2);
#endif
	return s1 == s2;
}

/* If needed for debug purposes, convert string_set_cmp() to strcmp() */
//#define string_set_cmp(s1, s2) (strcmp(s1, s2) == 0)

#endif /* _STRING_SET_H_ */
