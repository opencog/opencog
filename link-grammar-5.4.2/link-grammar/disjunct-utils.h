/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this softwares.   */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LINK_GRAMMAR_DISJUNCT_UTILS_H_
#define _LINK_GRAMMAR_DISJUNCT_UTILS_H_

#include <stdbool.h>

#include "api-types.h"

// Can undefine VERIFY_MATCH_LIST when done debugging...
#define VERIFY_MATCH_LIST

struct Disjunct_struct
{
	Disjunct *next;
	Connector *left, *right;
	double cost;
	bool marked;               /* unmarked disjuncts get deleted */

	/* match_left, right used only during parsing, for the match list. */
	bool match_left, match_right;

#ifdef VERIFY_MATCH_LIST
	int match_id;              /* verify the match list integrity */
#endif
	gword_set *originating_gword; /* Set of originating gwords */
	const char * word_string;     /* subscripted dictionary word */
};

/* Disjunct utilities ... */
void free_disjuncts(Disjunct *);
unsigned int count_disjuncts(Disjunct *);
Disjunct * catenate_disjuncts(Disjunct *, Disjunct *);
Disjunct * eliminate_duplicate_disjuncts(Disjunct * );
char * print_one_disjunct(Disjunct *);
void word_record_in_disjunct(const Gword *, Disjunct *);
Disjunct * disjuncts_dup(Disjunct *origd);

#endif /* _LINK_GRAMMAR_DISJUNCT_UTILS_H_ */
