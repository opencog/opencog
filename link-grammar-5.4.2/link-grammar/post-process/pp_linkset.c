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

/***********************************************************************
pp_linkset.c
maintains sets of pointers to link names
Similar to string-set, except that the comparison and hashing functions are
tailored for links. More importantly, all we store here is pointers. It's up
to the caller to ensure that the pointers always point to something useful.
**********************************************************************/

#include <memory.h>

#include "post-process.h"
#include "pp_linkset.h"
#include "utilities.h"

#define LINKSET_SPARSENESS 2
#define LINKSET_SEED_VALUE 37

static void clear_hash_table(pp_linkset *ls)
{
	memset(ls->hash_table,0,ls->hash_table_size*sizeof(pp_linkset_node *));
}

static void initialize(pp_linkset *ls, int size)
{
	ls->hash_table_size = size*LINKSET_SPARSENESS;
	ls->population = 0;
	ls->hash_table =
		(pp_linkset_node**) xalloc (ls->hash_table_size*sizeof(pp_linkset_node *));
	clear_hash_table(ls);
}

static unsigned int compute_hash(pp_linkset *ls, const char *str)
{
	/* hash is computed from capitalized prefix only */
	unsigned int i, hashval;
	hashval = LINKSET_SEED_VALUE;
	i = 0;
	if (islower((int)str[0])) i++; /* skip head-dependent indicator */
	for (; isupper((int)str[i]); i++)
		hashval = str[i] + 31*hashval;
	hashval %= ls->hash_table_size;
	return hashval;
}

static pp_linkset_node *add_internal(pp_linkset *ls, const char *str)
{
	pp_linkset_node *p, *n;
	unsigned int hashval;

	/* look for str (exactly) in linkset */
	hashval = compute_hash(ls, str);
	for (p=ls->hash_table[hashval]; p!=0; p=p->next)
		if (!strcmp(p->str,str)) return NULL;  /* already present */

	/* create a new node for u; stick it at head of linked list */
	n = (pp_linkset_node *) xalloc (sizeof(pp_linkset_node));
	n->next = ls->hash_table[hashval];
	n->str = str;
	ls->hash_table[hashval] = n;
	return n;
}

pp_linkset *pp_linkset_open(int size)
{
	pp_linkset *ls;
	if (size==0) return NULL;
	ls = (pp_linkset *) xalloc (sizeof(pp_linkset));
	initialize(ls, size);
	return ls;
}

void pp_linkset_close(pp_linkset *ls)
{
	if (ls == NULL) return;
	pp_linkset_clear(ls);      /* free memory taken by linked lists */
	xfree((void*) ls->hash_table, ls->hash_table_size*sizeof(pp_linkset_node*));
	xfree((void*) ls, sizeof(pp_linkset));
}

void pp_linkset_clear(pp_linkset *ls)
{
	/* clear dangling linked lists, but retain hash table itself */
	unsigned int i;
	pp_linkset_node *p;
	if (ls == NULL) return;
	for (i=0; i<ls->hash_table_size; i++)
	{
		p = ls->hash_table[i];
		while (p)
		{
			pp_linkset_node *q = p;
			p = p->next;
			xfree((void*) q, sizeof(pp_linkset_node));
		}
	}
	clear_hash_table(ls);
	ls->population = 0;
}

/**
 * returns 0 if already there, 1 if new. Stores only the pointer
 */
bool pp_linkset_add(pp_linkset *ls, const char *str)
{
	assert(ls != NULL, "pp_linkset internal error: Trying to add to a null set");

	if (add_internal(ls, str) == NULL) return false;
	ls->population++;
	return true;
}

/**
 * Set query. Returns 1 if str pp-matches something in the set, 0 otherwise
 */
bool pp_linkset_match(pp_linkset *ls, const char *str)
{
	int hashval;
	pp_linkset_node *p;
	if (ls == NULL) return false;
	hashval = compute_hash(ls, str);
	p = ls->hash_table[hashval];
	while (p != 0)
	{
		if (post_process_match(p->str, str)) return true;
		p = p->next;
	}
	return false;
}

bool pp_linkset_match_bw(pp_linkset *ls, const char *str)
{
	unsigned int hashval;
	pp_linkset_node *p;
	if (ls == NULL) return false;
	hashval = compute_hash(ls, str);
	p = ls->hash_table[hashval];
	while (p != 0)
	{
		if (post_process_match(str, p->str)) return true;
		p = p->next;
	}
	return false;
}

size_t pp_linkset_population(pp_linkset *ls)
{
	return (ls == NULL) ? 0 : ls->population;
}

