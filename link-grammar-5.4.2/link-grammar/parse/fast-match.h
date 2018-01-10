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

#ifndef _FAST_MATCH_H_
#define _FAST_MATCH_H_

#include <stddef.h> // for size_t
#include "api-types.h"
#include "link-includes.h" // for Sentence

typedef struct Match_node_struct Match_node;
struct Match_node_struct
{
	Match_node * next;
	Disjunct * d;
};

typedef struct fast_matcher_s fast_matcher_t;
struct fast_matcher_s
{
	size_t size;
	unsigned int *l_table_size;  /* the sizes of the hash tables */
	unsigned int *r_table_size;

	/* the beginnings of the hash tables */
	Match_node *** l_table;
	Match_node *** r_table;

	/* I'll pedantically maintain my own array of these cells */
	Disjunct ** match_list;      /* match-list stack */
	size_t match_list_end;       /* index to the match-list stack end */
	size_t match_list_size;      /* number of allocated elements */
};

/* See the source file for documentation. */
fast_matcher_t* alloc_fast_matcher(const Sentence);
void free_fast_matcher(fast_matcher_t*);

size_t form_match_list(fast_matcher_t *, int, Connector *, int, Connector *, int);

/**
 * Return the match-list element at the given index.
 */
static inline Disjunct *get_match_list_element(fast_matcher_t *ctxt, size_t mli)
{
	return ctxt->match_list[mli];
}

/**
 * Pop up the match-list stack
 */
static inline void pop_match_list(fast_matcher_t *ctxt, size_t match_list_last)
{
	ctxt->match_list_end = match_list_last;
}

#endif
