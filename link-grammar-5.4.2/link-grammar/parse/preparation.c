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

#include "api-structures.h"
#include "prepare/build-disjuncts.h"
#include "connectors.h"
#include "dict-common/dict-common.h" // For Dictionary_s
#include "disjunct-utils.h"
#include "externs.h"
#include "preparation.h"
#include "print/print.h"
#include "prune.h"
#include "resources.h"
#include "string-set.h"
#include "tokenize/word-structures.h" // for Word_struct

static void
set_connector_list_length_limit(Connector *c,
                                Connector_set *conset,
                                int short_len,
                                bool all_short,
                                const char * ZZZ)
{
	for (; c!=NULL; c=c->next)
	{
		if (string_set_cmp (ZZZ, c->string))
		{
			c->length_limit = 1;
		}
		else if (all_short ||
		         (conset != NULL && !match_in_connector_set(conset, c)))
		{
			c->length_limit = short_len;
		}
	}
}

static void
set_connector_length_limits(Sentence sent, Parse_Options opts)
{
	size_t i;
	unsigned int len = opts->short_length;
	bool all_short = opts->all_short;
	Connector_set * ucs = sent->dict->unlimited_connector_set;
	const char * ZZZ = string_set_add("ZZZ", sent->dict->string_set);

	if (0)
	{
		/* Not setting the length_limit saves observable time. However, if we
		 * would like to set the ZZZ connector length_limit to 1 for all
		 * sentences, we cannot do the following. */
		if (len >= sent->length) return; /* No point to enforce short_length. */
	}

	if (len > UNLIMITED_LEN) len = UNLIMITED_LEN;

	for (i=0; i<sent->length; i++)
	{
		Disjunct *d;
		for (d = sent->word[i].d; d != NULL; d = d->next)
		{
			set_connector_list_length_limit(d->left, ucs, len, all_short, ZZZ);
			set_connector_list_length_limit(d->right, ucs, len, all_short, ZZZ);
		}
	}
}


/**
 * Set c->nearest_word to the nearest word that this connector could
 * possibly connect to.  The connector *might*, in the end,
 * connect to something more distant, but this is the nearest
 * one that could be connected.
 */
static int set_dist_fields(Connector * c, size_t w, int delta)
{
	int i;
	if (c == NULL) return (int) w;
	i = set_dist_fields(c->next, w, delta) + delta;
	c->nearest_word = i;
	return i;
}

/**
 * Initialize the word fields of the connectors, and
 * eliminate those disjuncts that are so long, that they
 * would need to connect past the end of the sentence.
 */
static void setup_connectors(Sentence sent)
{
	size_t w;
	Disjunct * d, * xd, * head;
	for (w=0; w<sent->length; w++)
	{
		head = NULL;
		for (d=sent->word[w].d; d!=NULL; d=xd)
		{
			xd = d->next;
			if ((set_dist_fields(d->left, w, -1) < 0) ||
			    (set_dist_fields(d->right, w, 1) >= (int) sent->length))
			{
				d->next = NULL;
				free_disjuncts(d);
			}
			else
			{
				d->next = head;
				head = d;
			}
		}
		sent->word[w].d = head;
	}
}

/**
 * Record the wordgraph word in each of its connectors.
 * It is used for checking alternatives consistency.
 */
static void gword_record_in_connector(Sentence sent)
{
	for (size_t w = 0; w < sent->length; w++)
	{
		for (Disjunct *d = sent->word[w].d; d != NULL; d = d->next)
		{
			for (Connector *c = d->right; NULL != c; c = c->next)
				c->originating_gword = d->originating_gword;
			for (Connector *c = d->left; NULL != c; c = c->next)
				c->originating_gword = d->originating_gword;
		}
	}
}

/**
 * Turn sentence expressions into disjuncts.
 * Sentence expressions must have been built, before calling this routine.
 */
static void build_sentence_disjuncts(Sentence sent, double cost_cutoff)
{
	Disjunct * d;
	X_node * x;
	size_t w;
	for (w = 0; w < sent->length; w++)
	{
		d = NULL;
		for (x = sent->word[w].x; x != NULL; x = x->next)
		{
			Disjunct *dx = build_disjuncts_for_exp(x->exp, x->string, cost_cutoff);
			word_record_in_disjunct(x->word, dx);
			d = catenate_disjuncts(dx, d);
		}
		sent->word[w].d = d;
	}
}

/**
 * Assumes that the sentence expression lists have been generated.
 */
void prepare_to_parse(Sentence sent, Parse_Options opts)
{
	size_t i;

	build_sentence_disjuncts(sent, opts->disjunct_cost);
	if (verbosity_level(5))
	{
		printf("After expanding expressions into disjuncts:\n");
		print_disjunct_counts(sent);
	}
	print_time(opts, "Built disjuncts");

	for (i=0; i<sent->length; i++)
	{
		sent->word[i].d = eliminate_duplicate_disjuncts(sent->word[i].d);

		/* Some long Russian sentences can really blow up, here. */
		if (resources_exhausted(opts->resources))
			return;
	}
	print_time(opts, "Eliminated duplicate disjuncts");

	if (verbosity_level(5))
	{
		printf("\nAfter expression pruning and duplicate elimination:\n");
		print_disjunct_counts(sent);
	}

	gword_record_in_connector(sent);
	set_connector_length_limits(sent, opts);
	setup_connectors(sent);
}
