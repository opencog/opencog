/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2014 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LINKAGE_H
#define _LINKAGE_H

#include <stdbool.h>
#include "api-types.h"
#include "link-includes.h" // Needed for typedef WordIdx

/**
 * This summarizes the linkage status.
 */
struct Linkage_info_struct
{
	int index;            /* Index into the parse_set */
	bool discarded;
	short N_violations;
	short unused_word_cost;
	short link_cost;

	double disjunct_cost;
	double corpus_cost;
	const char *pp_violation_msg;
};

/**
 * num_links:
 *   The number of links in the current linkage.  Computed by
 *   extract_linkage().
 *
 * chosen_disjuncts[]
 *   This is an array pointers to disjuncts, one for each word, that is
 *   computed by extract_links().  It represents the chosen disjuncts
 *   for the current linkage.  It is used to compute the cost of the
 *   linkage, and also by compute_chosen_words() to compute the
 *   chosen_words[].
 *
 * link_array[]
 *   This is an array of links.  These links define the current linkage.
 *   It is computed by extract_links().  It is used by analyze_linkage().
 */
struct Linkage_s
{
	WordIdx         num_words;    /* Number of (tokenized) words */
	bool            is_sent_long; /* num_words >= twopass_length */
	const char *  * word;         /* Array of word spellings */

	size_t          num_links;    /* Number of links in array */
	Link *          link_array;   /* Array of links */
	size_t          lasz;         /* Alloc'ed length of link_array */

	Disjunct **     chosen_disjuncts; /* Disjuncts used, one per word */
	size_t          cdsz;         /* Alloc'ed length of chosen_disjuncts */
	char **         disjunct_list_str; /* Stringified version of above */
#ifdef USE_CORPUS
	Sense **        sense_list;   /* Word senses, inferred from disjuncts */
#endif

	Gword **wg_path;              /* Linkage Wordgraph path */
	Gword **wg_path_display;      /* Wordgraph path after morpheme combining */

	Linkage_info    lifo;         /* Parse_set index and cost information */
	PP_info *       pp_info;      /* PP domain info, one for each link */

	Sentence        sent;         /* Used for common linkage data */
};

struct Link_s
{
	size_t lw;              /* Offset into Linkage->word NOT Sentence->word */
	size_t rw;              /* Offset into Linkage->word NOT Sentence->word */
	Connector * lc;
	Connector * rc;
	const char * link_name; /* Spelling of full link name */
};

void compute_chosen_words(Sentence, Linkage, Parse_Options);

void partial_init_linkage(Sentence, Linkage, unsigned int N_words);
void check_link_size(Linkage);
void remove_empty_words(Linkage);
void free_linkage(Linkage);
void free_linkages(Sentence);

#endif /* _LINKAGE_H */
