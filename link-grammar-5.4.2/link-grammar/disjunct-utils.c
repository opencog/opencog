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

#include <string.h>
#include "connectors.h"
#include "disjunct-utils.h"
#include "print/print-util.h"
#include "string-set.h"
#include "utilities.h"
#include "tokenize/tok-structures.h" // XXX TODO provide gword access methods!

/* Disjunct utilities ... */

/**
 * free_disjuncts() -- free the list of disjuncts pointed to by c
 * (does not free any strings)
 */
void free_disjuncts(Disjunct *c)
{
	Disjunct *c1;
	for (;c != NULL; c = c1) {
		c1 = c->next;
		free_connectors(c->left);
		free_connectors(c->right);
		xfree((char *)c, sizeof(Disjunct));
	}
}

/**
 * Destructively catenates the two disjunct lists d1 followed by d2.
 * Doesn't change the contents of the disjuncts.
 * Traverses the first list, but not the second.
 */
Disjunct * catenate_disjuncts(Disjunct *d1, Disjunct *d2)
{
	Disjunct * dis = d1;

	if (d1 == NULL) return d2;
	if (d2 == NULL) return d1;
	while (dis->next != NULL) dis = dis->next;
	dis->next = d2;
	return d1;
}

/** Returns the number of disjuncts in the list pointed to by d */
unsigned int count_disjuncts(Disjunct * d)
{
	unsigned int count = 0;
	for (; d != NULL; d = d->next)
	{
		count++;
	}
	return count;
}

/* ============================================================= */

typedef struct disjunct_dup_table_s disjunct_dup_table;
struct disjunct_dup_table_s
{
	size_t dup_table_size;
	Disjunct ** dup_table;
};

/**
 * This is a hash function for disjuncts
 *
 * This is the old version that doesn't check for domination, just
 * equality.
 */
static inline unsigned int old_hash_disjunct(disjunct_dup_table *dt, Disjunct * d)
{
	Connector *e;
	unsigned int i;
	i = 0;
	for (e = d->left ; e != NULL; e = e->next) {
		i += string_hash(e->string);
	}
	for (e = d->right ; e != NULL; e = e->next) {
		i += string_hash(e->string);
	}
	i += string_hash(d->word_string);
	i += (i>>10);
	return (i & (dt->dup_table_size-1));
}

/**
 * The connectors must be exactly equal.  A similar function
 * is connectors_equal_AND(), but that ignores priorities,
 * this does not.
 */
static bool connectors_equal_prune(Connector *c1, Connector *c2)
{
	return string_set_cmp(c1->string, c2->string) && (c1->multi == c2->multi);
}

/** returns TRUE if the disjuncts are exactly the same */
static bool disjuncts_equal(Disjunct * d1, Disjunct * d2)
{
	Connector *e1, *e2;

	e1 = d1->left;
	e2 = d2->left;
	while ((e1 != NULL) && (e2 != NULL)) {
		if (!connectors_equal_prune(e1, e2)) return false;
		e1 = e1->next;
		e2 = e2->next;
	}
	if ((e1 != NULL) || (e2 != NULL)) return false;

	e1 = d1->right;
	e2 = d2->right;
	while ((e1 != NULL) && (e2 != NULL)) {
		if (!connectors_equal_prune(e1, e2)) return false;
		e1 = e1->next;
		e2 = e2->next;
	}
	if ((e1 != NULL) || (e2 != NULL)) return false;

	/* Save CPU time by comparing this last, since this will
	 * almost always be true. Rarely, the strings are not from
	 * the same string_set and hence the 2-step comparison. */
	if (d1->word_string == d2->word_string) return true;
	return (strcmp(d1->word_string, d2->word_string) == 0);
}

/**
 * Duplicate the given connector chain.
 * If the argument is NULL, return NULL.
 */
static Connector *connectors_dup(Connector *origc)
{
	Connector head;
	Connector *prevc = &head;
	Connector *newc = &head;
	Connector *t;

	for (t = origc; t != NULL;  t = t->next)
	{
		newc = connector_new();
		*newc = *t;

		prevc->next = newc;
		prevc = newc;
	}
	newc->next = NULL;

	return head.next;
}

/**
 * Duplicate the given disjunct chain.
 * If the argument is NULL, return NULL.
 */
Disjunct *disjuncts_dup(Disjunct *origd)
{
	Disjunct head;
	Disjunct *prevd = &head;
	Disjunct *newd = &head;
	Disjunct *t;

	for (t = origd; t != NULL; t = t->next)
	{
		newd = (Disjunct *)xalloc(sizeof(Disjunct));
		newd->word_string = t->word_string;
		newd->cost = t->cost;
		newd->left = connectors_dup(t->left);
		newd->right = connectors_dup(t->right);
		newd->originating_gword = t->originating_gword;
		prevd->next = newd;
		prevd = newd;
	}
	newd->next = NULL;

	return head.next;
}

static disjunct_dup_table * disjunct_dup_table_new(size_t sz)
{
	size_t i;
	disjunct_dup_table *dt;
	dt = (disjunct_dup_table *) xalloc(sizeof(disjunct_dup_table));

	dt->dup_table_size = sz;
	dt->dup_table = (Disjunct **) xalloc(sz * sizeof(Disjunct *));

	for (i=0; i<sz; i++) dt->dup_table[i] = NULL;

	return dt;
}

static void disjunct_dup_table_delete(disjunct_dup_table *dt)
{
	xfree(dt->dup_table, dt->dup_table_size * sizeof(Disjunct *));
	xfree(dt, sizeof(disjunct_dup_table));
}

#ifdef DEBUG
GNUC_UNUSED static int gword_set_len(const gword_set *gl)
{
	int len = 0;
	for (; NULL != gl; gl = gl->next) len++;
	return len;
}
#endif

/**
 * Return a new gword_set element, initialized from the given element.
 * @old_e Existing element.
 */
static gword_set *gword_set_element_new(gword_set *old_e)
{
	gword_set *new_e = malloc(sizeof(gword_set));
	*new_e = (gword_set){0};

	new_e->o_gword = old_e->o_gword;
	gword_set *chain_next = old_e->chain_next;
	old_e->chain_next = new_e;
	new_e->chain_next = chain_next;

	return new_e;
}

/**
 * Add an element to existing gword_set. Uniqueness is assumed.
 * @return A new set with the element.
 */
static gword_set *gword_set_add(gword_set *gset, gword_set *ge)
{
	gword_set *n = gword_set_element_new(ge);
	n->next = gset;
	gset = n;

	return gset;
}

/**
 * Combine the given gword sets.
 * The gword sets are not modified.
 * This function is used for adding the gword pointers of an eliminated
 * disjunct to the ones of the kept disjuncts, with no duplicates.
 *
 * @kept gword_set of the kept disjunct.
 * @eliminated gword_set of the eliminated disjunct.
 * @return Use copy-on-write semantics - the gword_set of the kept disjunct
 * just gets returned if there is nothing to add to it. Else - a new gword
 * set is returned.
 */
static gword_set *gword_set_union(gword_set *kept, gword_set *eliminated)
{
	/* Preserve the gword pointers of the eliminated disjunct if different. */
	gword_set *preserved_set = NULL;
	for (gword_set *e = eliminated; NULL != e; e = e->next)
	{
		gword_set *k;

		/* Ensure uniqueness. */
		for (k = kept; NULL != k; k = k->next)
			if (e->o_gword == k->o_gword) break;
		if (NULL != k) continue;

		preserved_set = gword_set_add(preserved_set, e);
	}

	if (preserved_set)
	{
		/* Preserve the originating gword pointers of the remaining disjunct. */
		for (gword_set *k = kept; NULL != k; k = k->next)
			preserved_set = gword_set_add(preserved_set, k);
		kept = preserved_set;
	}

	return kept;
}

/**
 * Takes the list of disjuncts pointed to by d, eliminates all
 * duplicates, and returns a pointer to a new list.
 * It frees the disjuncts that are eliminated.
 */
Disjunct * eliminate_duplicate_disjuncts(Disjunct * d)
{
	unsigned int i, h, count;
	Disjunct *dn, *dx;
	disjunct_dup_table *dt;

	count = 0;
	dt = disjunct_dup_table_new(next_power_of_two_up(2 * count_disjuncts(d)));

	while (d != NULL)
	{
		dn = d->next;
		h = old_hash_disjunct(dt, d);

		for (dx = dt->dup_table[h]; dx != NULL; dx = dx->next)
		{
			if (disjuncts_equal(dx, d)) break;
		}
		if (dx == NULL)
		{
			d->next = dt->dup_table[h];
			dt->dup_table[h] = d;
		}
		else
		{
			d->next = NULL;  /* to prevent it from freeing the whole list */
			if (d->cost < dx->cost) dx->cost = d->cost;

			dx->originating_gword =
				gword_set_union(dx->originating_gword, d->originating_gword);

			free_disjuncts(d);
			count++;
		}
		d = dn;
	}

	/* d is already null */
	for (i=0; i < dt->dup_table_size; i++)
	{
		for (dn = dt->dup_table[i]; dn != NULL; dn = dx) {
			dx = dn->next;
			dn->next = d;
			d = dn;
		}
	}

	lgdebug(+5+(0==count)*1000, "Killed %u duplicates\n", count);

	disjunct_dup_table_delete(dt);
	return d;
}

/* ============================================================= */

/* Return the stringified disjunct.
 * Be sure to free the string upon return.
 */

static void prt_con(Connector *c, dyn_str * p, char dir)
{
	if (NULL == c) return;
	prt_con (c->next, p, dir);

	if (c->multi)
	{
		append_string(p, "@%s%c ", c->string, dir);
	}
	else
	{
		append_string(p, "%s%c ", c->string, dir);
	}
}

char * print_one_disjunct(Disjunct *dj)
{
	dyn_str *p = dyn_str_new();

	prt_con(dj->left, p, '-');
	prt_con(dj->right, p, '+');

	return dyn_str_take(p);
}

/* ============================================================= */

/**
 * Record the wordgraph word to which the X-node belongs, in each of its
 * disjuncts.
 */
void word_record_in_disjunct(const Gword * gw, Disjunct * d)
{
	for (;d != NULL; d=d->next) {
		d->originating_gword = (gword_set *)&gw->gword_set_head;
	}
}
/* ========================= END OF FILE ============================== */
