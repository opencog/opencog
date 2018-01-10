/*************************************************************************/
/* Copyright (c) 2008, 2009, 2014 Linas Vepstas                          */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/
/*
 * lisjuncts.c
 *
 * Miscellaneous utilities for returning the list of disjuncts that
 * were actually used in a given parse of a sentence.
 */

#include <stdlib.h>
#include <string.h>
#include "api-structures.h"
#include "connectors.h"
#include "disjunct-utils.h"
#include "linkage.h"
#include "lisjuncts.h"

/* Links are *always* less than 10 chars long . For now. The estimate
 * below is somewhat dangerous .... could be  fixed. */
#define MAX_LINK_NAME_LENGTH 10

/**
 * Print connector list to string.
 * This reverses the order of the connectors in the connector list,
 * so that the resulting list is in the same order as it would appear
 * in the dictionary. The character 'dir' is appended to each connector.
 */
static char * reversed_conlist_str(Connector* c, char dir, char* buf, size_t sz)
{
	char* p;
	size_t len = 0;

	if (NULL == c) return buf;
	p = reversed_conlist_str(c->next, dir, buf, sz);

	sz -= (p-buf);

	if (c->multi)
		p[len++] = '@';

	len += lg_strlcpy(p+len, c->string, sz-len);
	if (3 < sz-len)
	{
		p[len++] = dir;
		p[len++] = ' ';
		p[len] = 0x0;
	}
	return p+len;
}

/**
 * Print disjunct to string.  The resulting list is in the same order
 * as it would appear in the dictionary.
 */
static void disjunct_str(Disjunct* dj, char* buf, size_t sz)
{
	char* p;
	if (NULL == dj) { *buf = 0; return; }
	p = reversed_conlist_str(dj->left, '-', buf, sz);
	reversed_conlist_str(dj->right, '+', p, sz - (p-buf));
}

/**
 * lg_compute_disjunct_strings -- Given sentence, compute disjuncts.
 *
 * This routine will compute the string representation of the disjunct
 * used for each word in parsing the given sentence. A string
 * representation of the disjunct is needed for most of the corpus
 * statistics functions: this string, together with the subscripted
 * word, is used as a key to index the statistics information in the
 * database.
 */
void lg_compute_disjunct_strings(Linkage lkg)
{
	char djstr[MAX_LINK_NAME_LENGTH*20]; /* no word will have more than 20 links */
	size_t nwords = lkg->num_words;

	if (lkg->disjunct_list_str) return;
	lkg->disjunct_list_str = (char **) malloc(nwords * sizeof(char *));
	memset(lkg->disjunct_list_str, 0, nwords * sizeof(char *));

	for (WordIdx w=0; w< nwords; w++)
	{
		Disjunct* dj = lkg->chosen_disjuncts[w];
		disjunct_str(dj, djstr, sizeof(djstr));

		lkg->disjunct_list_str[w] = strdup(djstr);
	}
}
