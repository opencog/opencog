/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2013 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/
/*
 * Miscellaneous utilities for dealing with word types.
 */

#include <math.h>

#include "dict-common/dict-utils.h" // for size_of_expression()
#include "connectors.h"

/**
 * free_connectors() -- free the list of connectors pointed to by e
 * (does not free any strings)
 */
void free_connectors(Connector *e)
{
	Connector * n;
	for (; e != NULL; e = n)
	{
		n = e->next;
		xfree((char *)e, sizeof(Connector));
	}
}

Connector * connector_new(void)
{
	Connector *c = (Connector *) xalloc(sizeof(Connector));
	init_connector(c);
	c->nearest_word = 0;
	c->multi = false;
	c->lc_start = 0;
	c->uc_length = 0;
	c->uc_start = 0;
	c->next = NULL;
	c->string = "";
	c->tableNext = NULL;
	return c;
}

/* ======================================================== */
/* Connector-set utilities ... */
/**
 * This hash function only looks at the leading upper case letters of
 * the string, and the direction, '+' or '-'.
 */
static unsigned int connector_set_hash(Connector_set *conset, const char * s, int d)
{
	unsigned int i;
	if (islower((int)*s)) s++; /* skip head-dependent indicator */

	/* djb2 hash */
	i = 5381;
	i = ((i << 5) + i) + d;
	while (isupper((int) *s)) /* connector tables cannot contain UTF8, yet */
	{
		i = ((i << 5) + i) + *s;
		s++;
	}
	return (i & (conset->table_size-1));
}

static void build_connector_set_from_expression(Connector_set * conset, Exp * e)
{
	E_list * l;
	Connector * c;
	unsigned int h;
	if (e->type == CONNECTOR_type)
	{
		c = connector_new();
		c->string = e->u.string;
		h = connector_set_hash(conset, c->string, e->dir);
		c->next = conset->hash_table[h];
		conset->hash_table[h] = c;
	} else {
		for (l=e->u.l; l!=NULL; l=l->next) {
			build_connector_set_from_expression(conset, l->e);
		}
	}
}

Connector_set * connector_set_create(Exp *e)
{
	unsigned int i;
	Connector_set *conset;

	conset = (Connector_set *) xalloc(sizeof(Connector_set));
	conset->table_size = next_power_of_two_up(size_of_expression(e));
	conset->hash_table =
	  (Connector **) xalloc(conset->table_size * sizeof(Connector *));
	for (i=0; i<conset->table_size; i++) conset->hash_table[i] = NULL;
	build_connector_set_from_expression(conset, e);
	return conset;
}

void connector_set_delete(Connector_set * conset)
{
	unsigned int i;
	if (conset == NULL) return;
	for (i=0; i<conset->table_size; i++) free_connectors(conset->hash_table[i]);
	xfree(conset->hash_table, conset->table_size * sizeof(Connector *));
	xfree(conset, sizeof(Connector_set));
}

/**
 * Returns TRUE the given connector is in this conset.  FALSE otherwise.
 * d='+' means this connector is on the right side of the disjunct.
 * d='-' means this connector is on the left side of the disjunct.
 */

bool match_in_connector_set(Connector_set *conset, Connector * c)
{
	unsigned int h;
	Connector * c1;
	if (conset == NULL) return false;
	h = connector_set_hash(conset, c->string, '+');
	for (c1 = conset->hash_table[h]; c1 != NULL; c1 = c1->next)
	{
		if (easy_match(c1->string, c->string)) return true;
	}
	return false;
}

/* ======================================================== */

/**
 * This hash function only looks at the leading upper case letters of
 * the connector string, and the label fields.  This ensures that if two
 * strings match (formally), then they must hash to the same place.
 */
int calculate_connector_hash(Connector * c)
{
	const char *s;
	unsigned int i;

	/* For most situations, all three hashes are very nearly equal;
	 * as to which is faster depends on the parsed text.
	 * For both English and Russian, there are about 100 pre-defined
	 * connectors, and another 2K-4K autogen'ed ones (the IDxxx idiom
	 * connectors, and the LLxxx suffix connectors for Russian).
	 * Turns out the cost of setting up the hash table dominates the
	 * cost of collisions. */
#ifdef USE_DJB2
	/* djb2 hash */
	i = 5381;
	s = c->string;
	if (islower((int) *s)) s++; /* ignore head-dependent indicator */
	while (isupper((int) *s)) /* connector tables cannot contain UTF8, yet */
	{
		i = ((i << 5) + i) + *s;
		s++;
	}
	i += i>>14;
#endif /* USE_DJB2 */

#define USE_JENKINS
#ifdef USE_JENKINS
	/* Jenkins one-at-a-time hash */
	i = 0;
	s = c->string;
	if (islower((int) *s)) s++; /* ignore head-dependent indicator */
	c->uc_start = s - c->string;
	while (isupper((int) *s)) /* connector tables cannot contain UTF8, yet */
	{
		i += *s;
		i += (i<<10);
		i ^= (i>>6);
		s++;
	}
	i += (i << 3);
	i ^= (i >> 11);
	i += (i << 15);
#endif /* USE_JENKINS */

#ifdef USE_SDBM
	/* sdbm hash */
	i = 0;
	s = c->string;
	if (islower((int) *s)) s++; /* ignore head-dependent indicator */
	c->uc_start = s - c->string;
	while (isupper((int) *s))
	{
		i = *s + (i << 6) + (i << 16) - i;
		s++;
	}
#endif /* USE_SDBM */

	c->lc_start = ('\0' == *s) ? 0 : s - c->string;
	c->uc_length = s - c->string - c->uc_start;
	c->hash = i;
	return i;
}

/* ========================= END OF FILE ============================== */
