/*************************************************************************/
/* Copyright (c) 2009 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/
/*
 * expand.c
 *
 * Enlarge the range of possible disjunct to consider while parsing.
 */

#include "api-structures.h"
#include "connectors.h"
#include "dict-common/dict-common.h" // for X_node_struct
#include "disjunct-utils.h"
#include "expand.h"
#include "tokenize/word-structures.h" // For Word_struct
#include "corpus/cluster.h"

/* ========================================================= */

static Disjunct * build_expansion_disjuncts(Cluster *clu, const char *xstr)
{
	Disjunct *dj;
	dj = lg_cluster_get_disjuncts(clu, xstr);
	if (dj && (verbosity > 0)) prt_error("Expanded %s \n", xstr);
	return dj;
}

/**
 * Increase the number of disjuncts associated to each word in the
 * sentence by working with word-clusters. Return true if the number
 * of disjuncts were expanded, else return false.
 */
bool lg_expand_disjunct_list(Sentence sent)
{
	size_t w;

	Cluster *clu = lg_cluster_new();

	bool expanded = false;
	for (w = 0; w < sent->length; w++)
	{
		X_node * x;
		Disjunct * d = sent->word[w].d;
		for (x = sent->word[w].x; x != NULL; x = x->next)
		{
			Disjunct *dx = build_expansion_disjuncts(clu, x->string);
			if (dx)
			{
				unsigned int cnt = count_disjuncts(d);
				d = catenate_disjuncts(dx, d);
				d = eliminate_duplicate_disjuncts(d);
				if (cnt < count_disjuncts(d)) expanded = true;
			}
		}
		sent->word[w].d = d;
	}
	lg_cluster_delete(clu);

	return expanded;
}
