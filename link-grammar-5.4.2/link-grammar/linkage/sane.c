/*************************************************************************/
/* Copyright 2013, 2014 Linas Vepstas                                    */
/* Copyright 2014 Amir Plivatsky                                         */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include "api-structures.h"  // for Sentence_s
#include "api-types.h"
#include "dict-common/regex-morph.h" // for match_regex
#include "connectors.h" // for MAX_SENTENCE
#include "disjunct-utils.h"  // for Disjunct_struct
#include "lg_assert.h"
#include "linkage.h"
#include "sane.h"
#include "tokenize/tok-structures.h" // Needed for Wordgraph_pathpos_s
#include "tokenize/word-structures.h" // for Word_struct
#include "tokenize/wordgraph.h"
#include "utilities.h"

/**
 * Construct word paths (one or more) through the Wordgraph.
 *
 * Add 'current_word" to the potential path.
 * Add "p" to the path queue, which defines the start of the next potential
 * paths to be checked.
 *
 * Each path is up to the current word (not including). It doesn't actually
 * construct a full path if there are null words - they break it. The final path
 * is constructed when the Wordgraph termination word is encountered.
 *
 * Note: The final path doesn't match the linkage word indexing if the linkage
 * contains empty words, at least until empty words are eliminated from the
 * linkage (in compute_chosen_words()). Further processing of the path is done
 * there in case morphology splits are to be hidden or there are morphemes with
 * null linkage.
 */
#define D_WPA 7
static void wordgraph_path_append(Wordgraph_pathpos **nwp, const Gword **path,
                                  Gword *current_word, /* add to the path */
                                  Gword *p)      /* add to the path queue */
{
	size_t n = wordgraph_pathpos_len(*nwp);

	assert(NULL != p, "Tried to add a NULL word to the word queue");
	if (current_word == p)
	{
		lgdebug(D_WPA, "Adding the same word '%s' again\n", p->subword);
		//print_lwg_path((Gword **)path, "After adding the same word");
	}

	/* Check if the path queue already contains the word to be added to it. */
	const Wordgraph_pathpos *wpt = NULL;

	if (NULL != *nwp)
	{
		for (wpt = *nwp; NULL != wpt->word; wpt++)
		{
			if (p == wpt->word)
			{
				lgdebug(D_WPA, "Word %s (after %zu) exists (after %zu)\n",
				        p->subword,
				        wpt->path[gwordlist_len(wpt->path)-1]->sent_wordidx,
				        path[gwordlist_len(path)-1]->sent_wordidx);
				/* If we are here, there are 2 or more paths leading to this word
				 * (p) that end with the same number of consecutive null words that
				 * consist an entire alternative. These null words represent
				 * different ways to split the subword upward in the hierarchy.
				 * For a nicer result we choose the shorter path. */
				if (wpt->path[gwordlist_len(wpt->path)-1]->sent_wordidx <=
				    path[gwordlist_len(path)-1]->sent_wordidx)
				{
					lgdebug(D_WPA, "Shorter path already queued\n");
					return; /* The shorter path is already in the queue. */
				}
				lgdebug(D_WPA, "Longer path is in the queue\n");
				//print_lwg_path((Gword **)wpt->path, "Freeing");
				free(wpt->path); /* To be replaced by a shorter path. */
				break;
			}
		}
	}

	if ((NULL == wpt) || (p != wpt->word))
	{
		/* Not already in the path queue - add it. */
		*nwp = wordgraph_pathpos_resize(*nwp, n+1);
	}
	else
	{
		lgdebug(D_WPA, "Path position to be replaced (len %zu): %zu\n", n,
		                wpt - *nwp);
		n = wpt - *nwp; /* Replace this path. */
	}
	(*nwp)[n].word = p;

	if (MT_INFRASTRUCTURE == p->prev[0]->morpheme_type)
	{
			/* Previous word is the Wordgraph dummy word. Initialize the path. */
			(*nwp)[n].path = NULL;
	}
	else
	{
		/* Duplicate the path from the current one. */

		size_t path_arr_size = (gwordlist_len(path)+1)*sizeof(*path);

		(*nwp)[n].path = malloc(path_arr_size);
		memcpy((*nwp)[n].path, path, path_arr_size);
	}

	/* If we queue the same word again, its path remains the same.
	 * Else append the current word to it. */
	if (p != current_word)
	{
		/* FIXME (cast) but anyway gwordlist_append() doesn't modify Gword. */
		gwordlist_append((Gword ***)&(*nwp)[n].path, current_word);
	}
}

/**
 * Free the Wordgraph paths and the Wordgraph_pathpos array.
 * In case of a match, the final path is still needed so this function is
 * then invoked with free_final_path=false.
 */
static void wordgraph_path_free(Wordgraph_pathpos *wp, bool free_final_path)
{
	Wordgraph_pathpos *twp;

	if (NULL == wp) return;
	for (twp = wp; NULL != twp->word; twp++)
	{
		if (free_final_path || (MT_INFRASTRUCTURE != twp->word->morpheme_type))
			free(twp->path);
	}
	free(wp);
}

#define NO_WORD (MAX_SENTENCE+1)

/**
 * Return the number of islands in a linkage.
 * First, each word appears in its own linked list.
 * Then all the links in the linkage are traversed, and the lists pointed
 * by each of them are combined.
 * Finally, the words are traversed and the lists are followed and
 * numbered. The WG path is used to skip optional words which are null.
 */
static size_t num_islands(const Linkage lkg, const Gword **wg_path)
{
	struct word
	{
		int prev;
		int next;
		int inum;
	};
	struct word *word = alloca(lkg->sent->length * sizeof(struct word));

	/* Initially, each word is in its own island. */
	for (WordIdx w = 0; w < lkg->sent->length; w++)
	{
		word[w].prev = word[w].next = w;
	}

	/* Unify the potential islands pointed by each link
	 * (if they are already unified, they remain so.) */
	for (LinkIdx li = 0; li < lkg->num_links; li++)
	{
		Link *l = &lkg->link_array[li];

		WordIdx iw;
		for (iw = word[l->lw].next; (iw != l->rw) && (iw != l->lw); iw = word[iw].next)
			;

		if (iw != l->rw)
		{
			int nextl = word[l->lw].next;
			int prevr = word[l->rw].prev;

			word[l->lw].next = l->rw;
			word[l->rw].prev = l->lw;

			word[prevr].next = nextl;
			word[nextl].prev = prevr;
		}

		if (verbosity_level(+8))
		{
			for (WordIdx w = 0; w < lkg->sent->length; w++)
			{
				err_msg(lg_Debug, "%d<-%zu->%d ", word[w].prev, w, word[w].next);
			}
			err_msg(lg_Debug, "\n");
		}
	}

	/* Count islands. */
	int inum = -1;
	Disjunct **cdj = lkg->chosen_disjuncts;

	for (WordIdx w = 0; w < lkg->sent->length; w++)
	{
		/* Skip null words which are optional words. */
		if ((NULL == *wg_path) || ((*wg_path)->sent_wordidx != w))
		{
			assert(word[w].prev == word[w].next);
			assert((NULL == cdj[w]) && lkg->sent->word[w].optional);

			word[w].prev = NO_WORD;
			word[w].inum = -1; /* not belonging to any island */
			continue;
		}

		wg_path++;
		if (NO_WORD == word[w].prev) continue;

		inum++;
		for (WordIdx iw = w; NO_WORD != word[iw].prev; iw = word[iw].next)
		{
			word[iw].prev = NO_WORD;
			word[iw].inum = inum;
		}
	}

	if (verbosity_level(8))
	{
		err_msg(lg_Debug, "Island count %d: ", inum);
		for (WordIdx w = 0; w < lkg->sent->length; w++)
		{
			err_msg(lg_Debug, "%d ", word[w].inum);
		}
		err_msg(lg_Debug, "\n");
	}

	return inum;
}

/* ============================================================== */
/* A kind of morphism post-processing */

/* These letters create a string that should be matched by a
 * SANEMORPHISM regex, given in the affix file. The empty word
 * doesn't have a letter. E.g. for the Russian dictionary: "w|ts".
 * It is converted here to: "^((w|ts)b)+$".
 * It matches "wbtsbwbtsbwb" but not "wbtsbwsbtsb".
 * FIXME? In this version of the function, 'b' is not yet supported,
 * so "w|ts" is converted to "^(w|ts)+$" for now.
 */
#define AFFIXTYPE_PREFIX   'p'   /* prefix */
#define AFFIXTYPE_STEM     't'   /* stem */
#define AFFIXTYPE_SUFFIX   's'   /* suffix */
#define AFFIXTYPE_MIDDLE   'm'   /* middle morpheme */
#define AFFIXTYPE_WORD     'w'   /* regular word */
#ifdef WORD_BOUNDARIES
#define AFFIXTYPE_END      'b'   /* end of input word */
#endif

/**
 * This routine solves the problem of mis-linked alternatives,
 * i.e a morpheme in one alternative that is linked to a morpheme in
 * another alternative. This can happen due to the way in which word
 * alternatives are implemented.
 *
 * It does so by checking that all the chosen disjuncts in a linkage
 * (including null words) match, in the same order, a path in the
 * Wordgraph.
 *
 * An important side effect of this check is that if the linkage is
 * good, its Wordgraph path is found.
 *
 * Optionally (if SANEMORPHISM regex is defined in the affix file), it
 * also validates that the morpheme-type sequence is permitted for the
 * language. This is a sanity check of the program and the dictionary.
 *
 * Return true if the linkage is good, else return false.
 */
#define D_SLM 8
bool sane_linkage_morphism(Sentence sent, Linkage lkg, Parse_Options opts)
{
	Wordgraph_pathpos *wp_new = NULL;
	Wordgraph_pathpos *wp_old = NULL;
	Wordgraph_pathpos *wpp;
	Gword **next; /* next Wordgraph words of the current word */
	size_t i;
	size_t null_count_found = 0;

	bool match_found = true; /* if all the words are null - it's still a match */
	Gword **lwg_path;

	Dictionary afdict = sent->dict->affix_table;       /* for SANEMORPHISM */
	char *const affix_types = alloca(sent->length*2 + 1);   /* affix types */
	affix_types[0] = '\0';

	lkg->wg_path = NULL;

	/* Populate the path word queue, initializing the path to NULL. */
	for (next = sent->wordgraph->next; *next; next++)
	{
		wordgraph_path_append(&wp_new, /*path*/NULL, /*add_word*/NULL, *next);
	}
	assert(NULL != wp_new, "Path word queue is empty");

	for (i = 0; i < lkg->num_words; i++)
	{
		Disjunct *cdj;            /* chosen disjunct */

		lgdebug(D_SLM, "lkg=%p Word %zu: ", lkg, i);

		if (NULL == wp_new)
		{
			lgdebug(D_SLM, "- No more words in the wordgraph\n");
			match_found = false;
			break;
		}

		if (wp_old != wp_new)
		{
			wordgraph_path_free(wp_old, true);
			wp_old = wp_new;
		}
		wp_new = NULL;
		//wordgraph_pathpos_print(wp_old);

		cdj = lkg->chosen_disjuncts[i];
		/* Handle null words */
		if (NULL == cdj)
		{
			lgdebug(D_SLM, "- Null word");
			/* A null word matches any word in the Wordgraph -
			 * so, unconditionally proceed in all paths in parallel. */
			match_found = false;
			bool optional_word_found = false;
			for (wpp = wp_old; NULL != wpp->word; wpp++)
			{
				if ((MT_INFRASTRUCTURE == wpp->word->morpheme_type) ||
				    (wpp->word->sent_wordidx > i))
				{
					assert(sent->word[i].optional, "wordindex=%zu", i);
					lgdebug(D_SLM, " (Optional, index=%zu)\n", i);
					// Retain the same word in the new path queue.
					wordgraph_path_append(&wp_new, wpp->path, wpp->word, wpp->word);
					match_found = true;
					optional_word_found = true;
					continue; /* Disregard this chosen disjunct. */
				}

				/* The null words cannot be marked here because wpp->path consists
				 * of pointers to the Wordgraph words, and these words are common to
				 * all the linkages, with potentially different null words in each
				 * of them. However, the position of the null words can be inferred
				 * from the null words in the word array of the Linkage structure.
				 */
				for (next = wpp->word->next; NULL != *next; next++)
				{
					if (MT_INFRASTRUCTURE != wpp->word->morpheme_type)
						match_found = true;
					wordgraph_path_append(&wp_new, wpp->path, wpp->word, *next);
				}
			}

			if (!optional_word_found)
			{
				null_count_found++;
				/* Note that if all the sentence words are null-words, its
				 * null_count is only sent->length-1 so this is not a mismatch. */
				if ((null_count_found > lkg->sent->null_count) &&
				    (lkg->sent->null_count != sent->length-1))
				{
					lgdebug(D_SLM, " (Extra, count > %zu)\n", lkg->sent->null_count);
					match_found = false;
					break;
				}
				lgdebug(D_SLM, "\n");
			}

			if (NULL != wpp->word) break; /* Extra null count */
			continue;
		}

		if (!match_found)
		{
			const char *e = "Internal error: Too many words in the linkage";
			lgdebug(D_SLM, "- %s\n", e);
			prt_error("Error: %s.\n", e);
			break;
		}

		if (verbosity_level(D_SLM)) prt_error("%s", cdj->word_string);

		match_found = false;
		/* Proceed in all the paths in which the word is found. */
		for (wpp = wp_old; NULL != wpp->word; wpp++)
		{
			for (gword_set *gl = cdj->originating_gword; NULL != gl; gl =  gl->next)
			{
				if (gl->o_gword == wpp->word)
				{
					match_found = true;
					for (next = wpp->word->next; NULL != *next; next++)
					{
						wordgraph_path_append(&wp_new, wpp->path, wpp->word, *next);
					}
					break;
				}
			}
		}

		if (!match_found)
		{
			/* FIXME? A message can be added here if there are too many words
			 * in the linkage (can happen only if there is an internal error). */
			lgdebug(D_SLM, "- No Wordgraph match\n");
			break;
		}
		lgdebug(D_SLM, "\n");
	}

	if (match_found)
	{
		match_found = false;
		/* Validate that there are no missing words in the linkage.
		 * It is so, if the dummy termination word is found in the
		 * new pathpos queue.
		 */
		if (NULL != wp_new)
		{
			for (wpp = wp_new; NULL != wpp->word; wpp++)
			{
				if (MT_INFRASTRUCTURE == wpp->word->morpheme_type) {
					match_found = true;
					/* Exit the loop with with wpp of the termination word. */
					break;
				}
			}
		}
		if (!match_found)
		    lgdebug(D_SLM, "%p Missing word(s) at the end of the linkage.\n", lkg);
	}

	/* Reject found null count that is not consistent with sent->null_count.
	 * Here islands_ok=1 is handled, and also a lower-than-expected null
	 * count when islands_ok=0. */
	if (match_found)
	{
		size_t count_found =
			opts->islands_ok ? num_islands(lkg, wpp->path) : null_count_found;

		if ((count_found != lkg->sent->null_count) &&
		    (lkg->sent->null_count != sent->length-1) && (count_found != sent->length))
		{
			lgdebug(D_SLM, "Null count mismatch: Found %zu != null_count %zu\n",
					  count_found, lkg->sent->null_count);
			match_found = false;
		}
	}

#define DEBUG_morpheme_type 0
	/* Check the morpheme type combination.
	 * If null_count > 0, the morpheme type combination may be invalid
	 * due to null subwords, so skip this check. */
	if (match_found && (0 == sent->null_count) &&
		(NULL != afdict) && (NULL != afdict->regex_root))
	{
		const Gword **w;
		char *affix_types_p = affix_types;

		/* Construct the affix_types string. */
#if DEBUG_morpheme_type
		print_lwg_path(wpp->path, "Linkage");
#endif
		i = 0;
		for (w = wpp->path; *w; w++)
		{
			i++;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
			switch ((*w)->morpheme_type)
			{
#pragma GCC diagnostic pop
				default:
					/* What to do with the rest? */
				case MT_WORD:
					*affix_types_p = AFFIXTYPE_WORD;
					break;
				case MT_PREFIX:
					*affix_types_p = AFFIXTYPE_PREFIX;
					break;
				case MT_STEM:
					*affix_types_p = AFFIXTYPE_STEM;
					break;
				case MT_MIDDLE:
					*affix_types_p = AFFIXTYPE_MIDDLE;
					break;
				case MT_SUFFIX:
					*affix_types_p = AFFIXTYPE_SUFFIX;
					break;
			}

#if DEBUG_morpheme_type
			lgdebug(D_SLM, "Word %zu: %s affixtype=%c\n",
			     i, (*w)->subword,  *affix_types_p);
#endif

			affix_types_p++;
		}
		*affix_types_p = '\0';

#ifdef WORD_BOUNDARIES /* not yet implemented */
		{
			const Gword *uw;

			/* If w is an "end subword", return its unsplit word, else NULL. */
			uw = word_boundary(w); /* word_boundary() unimplemented */

			if (NULL != uw)
			{
				*affix_types_p++ = AFFIXTYPE_END;
				lgdebug(D_SLM, "%p End of Gword %s\n", lkg, uw->subword);
			}
		}
#endif

		/* Check if affix_types is valid according to SANEMORPHISM. */
		if (('\0' != affix_types[0]) &&
		    (NULL == match_regex(afdict->regex_root, affix_types)))
		{
			/* Morpheme type combination is invalid */
			match_found = false;
			/* Notify to stdout, so it will be shown along with the result.
			 * XXX We should have a better way to notify. */
			if (0 < opts->verbosity)
				prt_error("Warning: Invalid morpheme type combination '%s'.\n"
				          "Run with !bad and !verbosity>"STRINGIFY(D_USER_MAX)
				          " to debug\n", affix_types);
		}
	}

	if (match_found) lwg_path = (Gword **)wpp->path; /* OK to modify */
	wordgraph_path_free(wp_old, true);
	wordgraph_path_free(wp_new, !match_found);

	if (match_found)
	{
		if ('\0' != affix_types[0])
		{
			lgdebug(D_SLM, "%p Morpheme type combination '%s'\n", lkg, affix_types);
		}
		lgdebug(+D_SLM-1, "%p SUCCEEDED\n", lkg);
		lkg->wg_path = lwg_path;
		return true;
	}

	/* Oh no ... invalid morpheme combination! */
	lgdebug(+D_SLM-1, "%p FAILED\n", lkg);
	return false;
}
#undef D_SLM
