/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2013 Linas Vepstas                                      */
/* Copyright (c) 2014 Amir Plivatsky                                     */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _TOK_STRUCTURES_H_
#define _TOK_STRUCTURES_H_

#include <stddef.h>
#include "api-types.h"
#include "link-includes.h"

/* conditional compiling flags */
#define INFIX_NOTATION
    /* If defined, then we're using infix notation for the dictionary */
    /* otherwise we're using prefix notation */

/* An ordered set of gword pointers, used to indicate the source gword
 * (Wordgraph word) of disjuncts and connectors. Usually it contains only
 * one element.  However, when a duplicate disjunct is eliminated (see
 * eliminate_duplicate_disjuncts()) and it originated from a different
 * gword (a relatively rare event) its gword is added to the gword_set of
 * the remaining disjunct. A set of 3 elements is extremely rare. The
 * original order is preserved, in a hope for better caching on
 * alternatives match checks in fast-match.c.
 *
 * Memory management:
 * A copy-on-write semantics is used when constructing a new gword_set.  It
 * means that all the gword sets with one element are shared.  These gword
 * sets are part of the Gword structure. Copied and added element are
 * alloc'ed and chained. The result is that the chain_next of the gword
 * sets that are part of each gword contains the list of alloc'ed elements -
 * to be used in gword_set_delete() called *only* in sentence_delete().
 * This ensures that the gword_set of connectors doesn't get stale when
 * their disjuncts are deleted and later restored in one-step parse when
 * min_null_count=0 and max_null count>0 (see classic_parse()).
 */
struct gword_set
{
	Gword *o_gword;
	struct gword_set *next;
	struct gword_set *chain_next;
};


typedef enum
{
	MT_INVALID,            /* Zero, to be changed to the correct type */
	MT_WORD,               /* Regular word */
	MT_FEATURE,            /* Pseudo morpheme, currently capitalization marks */
	MT_INFRASTRUCTURE,     /* Start and end Wordgraph pseudo-words */
	MT_WALL,               /* The LEFT-WALL and RIGHT-WALL pseudo-words */
	MT_EMPTY,              /* Empty word FIXME: Remove it. */
	MT_UNKNOWN,            /* Unknown word (FIXME? Unused) */
	/* Experimental for Semitic languages (yet unused) */
	MT_TEMPLATE,
	MT_ROOT,
	/* Experimental - for display purposes.
	 * MT_CONTR is now used in the tokenization step, see the comments there. */
	MT_CONTR,              /* Contracted part of a contraction (e.g. y', 's) */
	MT_PUNC,               /* Punctuation (yet unused) */
	/* We are not going to have >63 types up to here. */
	MT_STEM    = 1<<6,     /* Stem */
	MT_PREFIX  = 1<<7,     /* Prefix */
	MT_MIDDLE  = 1<<8,     /* Middle morpheme (yet unused) */
	MT_SUFFIX  = 1<<9      /* Suffix */
} Morpheme_type;
#define IS_REG_MORPHEME (MT_STEM|MT_PREFIX|MT_MIDDLE|MT_SUFFIX)

/* Word status */
/* - Tokenization */
#define WS_UNKNOWN (1<<0) /* Unknown word */
#define WS_REGEX   (1<<1) /* Matches a regex */
#define WS_SPELL   (1<<2) /* Result of a spell guess */
#define WS_RUNON   (1<<3) /* Separated from words run-on */
#define WS_HASALT  (1<<4) /* Has alternatives (one or more)*/
#define WS_UNSPLIT (1<<5) /* It's an alternative to itself as an unsplit word */
#define WS_INDICT  (1<<6) /* boolean_dictionary_lookup() is true */
#define WS_FIRSTUPPER (1<<7) /* Subword is the lc version of its unsplit_word.
                                The original word can be restored if needed
                                through this unsplit_word. */
/* - Post linkage stage. XXX Experimental. */
#define WS_PL      (1<<14) /* Post-Linkage, not belonging to tokenization */

#define WS_GUESS (WS_SPELL|WS_RUNON|WS_REGEX)

/*Only TS_DONE is actually used. */
typedef enum
{
	TS_INITIAL,
	TS_LR_STRIP,
	TS_AFFIX_SPLIT,
	TS_REGEX,
	TS_RUNON,
	TS_SPELL,
	TS_DONE                  /* Tokenization done */
} Tokenizing_step;

/* For the "guess" field of Gword_struct. */
typedef enum
{
	GM_REGEX = '!',
	GM_SPELL = '~',
	GM_RUNON = '&',
	GM_UNKNOWN = '?'
} Guess_mark;

#define MAX_SPLITS 10   /* See split_counter below */

struct Gword_struct
{
	const char *subword;
	const char *start;   /* subword start position. */
	const char *end;     /* subword end position. */

	Gword *unsplit_word; /* Upward-going co-tree */
	Gword **next;        /* Right-going tree */
	Gword **prev;        /* Left-going tree */
	Gword *chain_next;   /* Next word in the chain of all words */

	/* Disjuncts and connectors point back to their originating Gword(s). */
	gword_set gword_set_head;

	/* Used by sane_linkage_morphism() and remove_empty_words() for
	 * locating optional words that should be removed. */
	WordIdx sent_wordidx; /* Index in the 2D sentence word array. */

	/* For debug and inspiration. */
	const char *label;   /* Debug label - code locations of tokenization */
	size_t node_num;     /* For differentiating words with identical subwords,
	                        and for indicating the order in which word splits
                           have been done. Shown in the Wordgraph display and in
                           debug messages. Not used otherwise. Could have been
                           used for hier_position instead of pointers in order
                           to optimize its generation and comparison. */

	/* Tokenizer state */
	Tokenizing_step tokenizing_step;
	bool issued_unsplit; /* The word has been issued as an alternative to itself.
	                        It will become an actual alternative to itself only
	                        if it's not the sole alternative, in which case it
	                        will be marked with WS_UNSPLIT. */
	size_t split_counter; /* Incremented on splits. A word cannot split more than
	                         MAX_SPLITS times and a warning is issued then. */

	unsigned int status;         /* See WS_* */
	Morpheme_type morpheme_type; /* See MT_* */
	Gword *alternative_id;       /* Alternative start - a unique identifier of
	                                the alternative to which the word belongs. */
	const char *regex_name;      /* Subword matches this regex.
                                   FIXME? Extend for multiple regexes. */

	/* Only used by wordgraph_flatten() */
	const Gword **hier_position; /* Unsplit_word/alternative_id pointer list, up
                                   to the original sentence word. */
	size_t hier_depth;           /* Number of pointer pairs in hier_position */

	/* XXX Experimental. Only used after the linkage (by compute_chosen_words())
	 * for an element in the linkage display wordgraph path that represents
	 * a block of null words that are morphemes of the same word. */
	Gword **null_subwords;       /* Null subwords represented by this word */
};

/* Wordgraph path word-positions,
 * used in wordgraph_flatten() and sane_linkage_morphism().
 * FIXME Separate to two different structures. */
struct Wordgraph_pathpos_s
{
	Gword *word;      /* Position in the Wordgraph */
	/* Only for wordgraph_flatten(). */
	bool same_word;   /* Still the same word - issue an empty word */
	bool next_ok;     /* OK to proceed to the next Wordgraph word */
	bool used;        /* Debug - the word has been issued */
	/* Only for sane_morphism(). */
	const Gword **path; /* Linkage candidate wordgraph path */
};

#endif
