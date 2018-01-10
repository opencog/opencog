/*************************************************************************/
/* Copyright (c) 2014 Amir Plivatsky                                     */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LG_DICT_AFFIX_H_
#define  _LG_DICT_AFFIX_H_

#include "dict-common.h"

/* The functions here are intended for use by the tokenizer, only,
 * and pretty much no one else. If you are not the tokenizer, you
 * probably don't need these. */

/* Connector names for the affix class lists in the affix file */

typedef enum {
	AFDICT_RPUNC=1,
	AFDICT_LPUNC,
	AFDICT_MPUNC,
	AFDICT_UNITS,
	AFDICT_SUF,
	AFDICT_PRE,
	AFDICT_MPRE,
	AFDICT_QUOTES,
	AFDICT_BULLETS,
	AFDICT_INFIXMARK,
	AFDICT_STEMSUBSCR,
	AFDICT_SANEMORPHISM,

	/* The below are used only for random morphology via regex */
	AFDICT_REGPRE,
	AFDICT_REGMID,
	AFDICT_REGSUF,
	AFDICT_REGALTS,
	AFDICT_REGPARTS,

	/* Have to have one last entry, to get the array size correct */
	AFDICT_LAST_ENTRY,
	AFDICT_NUM_ENTRIES
} afdict_classnum;

#define AFDICT_CLASSNAMES1 \
	"invalid classname", \
	"RPUNC", \
	"LPUNC", \
	"MPUNC", \
	"UNITS", \
	"SUF",         /* SUF is used in the Russian dict */ \
	"PRE",         /* PRE is not used anywhere, yet... */ \
	"MPRE",        /* Multi-prefix, currently for Hebrew */ \
	"QUOTES", \
	"BULLETS", \
	"INFIXMARK",   /* Prepended to suffixes, appended to prefixes */ \
	"STEMSUBSCR",  /* Subscripts for stems */ \
	"SANEMORPHISM", /* Regex for sane_morphism() */

/* The regexes below are used only for random morphology generation */
#define AFDICT_CLASSNAMES2 \
	"REGPRE",      /* Regex for prefix */ \
	"REGMID",      /* Regex for middle parts */ \
	"REGSUF",      /* Regex for suffix */ \
	"REGALTS",     /* Min&max number of alternatives to issue for a word */\
	"REGPARTS",    /* Max number of word partitions */

#define AFDICT_CLASSNAMES AFDICT_CLASSNAMES1 AFDICT_CLASSNAMES2 "last classname"
#define AFCLASS(afdict, class) (&afdict->afdict_class[class])

/* Suffixes start with it.
 * This is needed to distinguish suffixes that were stripped off from
 * ordinary words that just happen to be the same as the suffix.
 * Kind-of a weird hack, but I'm not sure what else to do...
 * Similarly, prefixes end with it.
 */
#define INFIX_MARK(afdict) \
	((NULL == afdict) ? '\0' : (AFCLASS(afdict, AFDICT_INFIXMARK)->string[0][0]))


Afdict_class * afdict_find(Dictionary, const char *, bool);
bool is_stem(const char *);

#endif /* _LG_DICT_AFFIX_H_ */
