/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2009, 2013 Linas Vepstas                                */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LINK_GRAMMAR_CONNECTORS_H_
#define _LINK_GRAMMAR_CONNECTORS_H_

#include <ctype.h>   // for islower()
#include <stdbool.h>
#include <stdint.h>  // for uint8_t

#include "api-types.h"

/* MAX_SENTENCE cannot be more than 254, because word MAX_SENTENCE+1 is
 * BAD_WORD -- it is used to indicate that nothing can connect to this
 * connector, and this should fit in one byte (because the word field
 * of a connector is an uint8_t, see below).
 */
#define MAX_SENTENCE 254        /* Maximum number of words in a sentence */

/* On a 64-bit machine, this struct should be exactly 4*8=32 bytes long.
 * Lets try to keep it that way.
 */
struct Connector_struct
{
	int16_t hash;
	uint8_t length_limit;
	             /* If this is a length limited connector, this
	                gives the limit of the length of the link
	                that can be used on this connector.  Since
	                this is strictly a function of the connector
	                name, efficiency is the only reason to store
	                this.  If no limit, the value is set to 255. */
	uint8_t nearest_word;
	             /* The nearest word to my left (or right) that
	                this could ever connect to.  Computed by
	                setup_connectors() */
	bool multi;  /* TRUE if this is a multi-connector */
	uint8_t lc_start;     /* lc start position (or 0) - for match speedup. */
	uint8_t uc_length;    /* uc part length - for match speedup. */
	uint8_t uc_start;     /* uc start position - for match speedup. */
	Connector * next;
	const char * string; /* The connector name w/o the direction mark, e.g. AB */

	/* Hash table next pointer, used only during pruning. */
	union
	{
		Connector * tableNext;
		const gword_set *originating_gword;
	};
};

struct Connector_set_s
{
	Connector ** hash_table;
	unsigned int table_size;
};

static inline void connector_set_string(Connector *c, const char *s)
{
	c->string = s;
	c->hash = -1;
}
static inline const char * connector_get_string(Connector *c)
{
	return c->string;
}

/* Connector utilities ... */
Connector * connector_new(void);
void free_connectors(Connector *);

/* Length-limits for how far connectors can reach out. */
#define UNLIMITED_LEN 255
#define SHORT_LEN 6

static inline Connector * init_connector(Connector *c)
{
	c->hash = -1;
	c->length_limit = UNLIMITED_LEN;
	return c;
}

/* Connector-set utilities ... */
Connector_set * connector_set_create(Exp *e);
void connector_set_delete(Connector_set * conset);
bool match_in_connector_set(Connector_set*, Connector*);


/**
 * Returns TRUE if s and t match according to the connector matching
 * rules.  The connector strings must be properly formed, starting with
 * zero or one lower case letters, followed by one or more upper case
 * letters, followed by some other letters.
 *
 * The algorithm is symmetric with respect to a and b.
 *
 * Connectors starting with lower-case letters match ONLY if the initial
 * letters are DIFFERENT.  Otherwise, connectors only match if the
 * upper-case letters are the same, and the trailing lower case letters
 * are the same (or have wildcards).
 *
 * The initial lower-case letters allow an initial 'h' (denoting 'head
 * word') to match an initial 'd' (denoting 'dependent word'), while
 * rejecting a match 'h' to 'h' or 'd' to 'd'.  This allows the parser
 * to work with catena, instead of just links.
 */
static inline bool easy_match(const char * s, const char * t)
{
	char is = 0, it = 0;
	if (islower((int) *s)) { is = *s; s++; }
	if (islower((int) *t)) { it = *t; t++; }

	if (is != 0 && it != 0 && is == it) return false;

	while (isupper((int)*s) || isupper((int)*t))
	{
		if (*s != *t) return false;
		s++;
		t++;
	}

	while ((*s!='\0') && (*t!='\0'))
	{
		if ((*s == '*') || (*t == '*') || (*s == *t))
		{
			s++;
			t++;
		}
		else
			return false;
	}
	return true;
}


static inline int string_hash(const char *s)
{
	unsigned int i;

	/* djb2 hash */
	i = 5381;
	while (*s)
	{
		i = ((i << 5) + i) + *s;
		s++;
	}
	return i;
}

int calculate_connector_hash(Connector *);

static inline int connector_hash(Connector * c)
{
	if (-1 != c->hash) return c->hash;
	return calculate_connector_hash(c);
}

/**
 * hash function. Based on some tests, this seems to be an almost
 * "perfect" hash, in that almost all hash buckets have the same size!
 */
static inline unsigned int pair_hash(unsigned int table_size,
                            int lw, int rw,
                            const Connector *le, const Connector *re,
                            unsigned int cost)
{
	unsigned int i;

#if 0
	/* hash function. Based on some tests, this seems to be
	 * an almost "perfect" hash, in that almost all hash buckets
	 * have the same size! */
	i = 1 << cost;
	i += 1 << (lw % (log2_table_size-1));
	i += 1 << (rw % (log2_table_size-1));
	i += ((unsigned int) le) >> 2;
	i += ((unsigned int) le) >> log2_table_size;
	i += ((unsigned int) re) >> 2;
	i += ((unsigned int) re) >> log2_table_size;
	i += i >> log2_table_size;
#else
	/* sdbm-based hash */
	i = cost;
	i = lw + (i << 6) + (i << 16) - i;
	i = rw + (i << 6) + (i << 16) - i;
	i = ((int)(intptr_t)le) + (i << 6) + (i << 16) - i;
	i = ((int)(intptr_t)re) + (i << 6) + (i << 16) - i;
#endif

	return i & (table_size-1);
}
#endif /* _LINK_GRAMMAR_CONNECTORS_H_ */
