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

#include <stdarg.h>
#include "api-structures.h" // Needed for Parse_Options
#include "disjunct-utils.h" // Needed for Disjunct
#include "linkage.h"
#include "score.h"

/**
 * This function defines the cost of a link as a function of its length.
 */
static inline int cost_for_length(int length)
{
	return length-1;
}

/**
 * Computes the cost of the current parse of the current sentence,
 * due to the length of the links.
 */
static size_t compute_link_cost(Linkage lkg)
{
	size_t lcost, i;
	lcost =  0;
	for (i = 0; i < lkg->num_links; i++)
	{
		lcost += cost_for_length(lkg->link_array[i].rw - lkg->link_array[i].lw);
	}
	return lcost;
}

static int unused_word_cost(Linkage lkg)
{
	int lcost;
	size_t i;
	lcost =  0;
	for (i = 0; i < lkg->num_words; i++)
		lcost += (lkg->chosen_disjuncts[i] == NULL);
	return lcost;
}

/**
 * Computes the cost of the current parse of the current sentence
 * due to the cost of the chosen disjuncts.
 */
static double compute_disjunct_cost(Linkage lkg)
{
	size_t i;
	double lcost;
	lcost =  0.0;
	for (i = 0; i < lkg->num_words; i++)
	{
		if (lkg->chosen_disjuncts[i] != NULL)
			lcost += lkg->chosen_disjuncts[i]->cost;
	}
	return lcost;
}

/** Assign parse score (cost) to linkage, used for parse ranking. */
void linkage_score(Linkage lkg, Parse_Options opts)
{
	lkg->lifo.unused_word_cost = unused_word_cost(lkg);
	if (opts->use_sat_solver)
	{
		lkg->lifo.disjunct_cost = 0.0;
	}
	else
	{
		lkg->lifo.disjunct_cost = compute_disjunct_cost(lkg);
	}
	lkg->lifo.link_cost = compute_link_cost(lkg);
	lkg->lifo.corpus_cost = -1.0;

	lg_corpus_score(lkg);
}
