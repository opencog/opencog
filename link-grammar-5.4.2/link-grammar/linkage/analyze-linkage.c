/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2012, 2014 Linas Vepstas                                */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <ctype.h>
#include <string.h>

#include "analyze-linkage.h"
#include "connectors.h" // Needed for connector_get_string
#include "linkage.h"
#include "string-set.h"

/* Links are *always* less than 10 chars long. For now... */
#define MAX_LINK_NAME_LENGTH 10

/**
 * This returns a string that is the the GCD of the two given strings.
 * If the GCD is equal to one of them, a pointer to it is returned.
 * Otherwise a new string for the GCD is put in the string set.
 */
static const char * intersect_strings(String_set *sset, const char * s, const char * t)
{
	int i, j, d;
	const char *w, *s0;
	char u0[MAX_LINK_NAME_LENGTH];
	char *u;

	/* The head indicator is lower-case h, the dependent indicator is
	 * lower-case d. If they are present, skip them. The intersection
	 * cannot include them. */
	if (islower((int) *s)) s++;
	if (islower((int) *t)) t++;

	if (strcmp(s,t) == 0) return s;  /* would work without this */
	i = strlen(s);
	j = strlen(t);
	if (j > i) {
		w = s; s = t; t = w;
	}
	/* s is now the longer (at least not the shorter) string */
	u = u0;
	d = 0;
	s0 = s;
	while (*t != '\0') {
		if ((*s == *t) || (*t == '*')) {
			*u = *s;
		} else {
			assert(*s == '*', "Invalid intersection!");
			d++;
			*u = *t;
		}
		s++; t++; u++;
	}
	if (d == 0) {
		return s0;
	} else {
		strcpy(u, s);   /* get the remainder of s */
		return string_set_add(u0, sset);
	}
}

/**
 * The name of the link is set to be the GCD of the names of
 * its two endpoints. Must be called after each extract_links(),
 * etc. since that call issues a brand-new set of links into
 * parse_info.
 */
void compute_link_names(Linkage lkg, String_set *sset)
{
	size_t i;
	for (i = 0; i < lkg->num_links; i++)
	{
		lkg->link_array[i].link_name = intersect_strings(sset,
			connector_get_string(lkg->link_array[i].lc),
			connector_get_string(lkg->link_array[i].rc));
	}
}
