/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2013, 2014 Linas Vepstas                                */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LG_DICT_COMMON_H_
#define  _LG_DICT_COMMON_H_

#include "api-types.h" // for pp_knowledge
#include "dict-structures.h"
#include "utilities.h" // for locale_t

#define EMPTY_CONNECTOR "ZZZ"
#define UNLIMITED_CONNECTORS_WORD ("UNLIMITED-CONNECTORS")

/* Forward decls */
typedef struct Afdict_class_struct Afdict_class;
typedef struct Exp_list_s Exp_list;
typedef struct Regex_node_s Regex_node;

/* Used for memory management */
struct Exp_list_s
{
	Exp * exp_list;
};

typedef struct X_node_struct X_node;
struct X_node_struct
{
	const char * string;       /* the word itself */
	Exp * exp;
	X_node *next;
	const Gword *word;         /* originating Wordgraph word */
};

/* The regexes are stored as a linked list of the following nodes. */
struct Regex_node_s
{
	char *name;      /* The identifying name of the regex */
	char *pattern;   /* The regular expression pattern */
	bool neg;        /* Negate the match */
	void *re;        /* The compiled regex. void * to avoid
	                    having re library details invading the
	                    rest of the LG system; regex-morph.c
	                    takes care of all matching.
	                  */
	Regex_node *next;
};

struct Afdict_class_struct
{
	size_t mem_elems;     /* number of memory elements allocated */
	size_t length;        /* number of strings */
	char const ** string;
};

#define MAX_TOKEN_LENGTH 250     /* Maximum number of chars in a token */

struct Dictionary_s
{
	Dict_node *  root;
	Regex_node * regex_root;
	const char * name;
	const char * lang;
	const char * version;
	const char * locale;    /* Locale name */
	locale_t     lctype;    /* Locale argument for the *_l() functions */
	int          num_entries;

	bool         use_unknown_word;
	bool         unknown_word_defined;
	bool         left_wall_defined;
	bool         right_wall_defined;
	bool         shuffle_linkages;

	/* Affixes are used during the tokenization stage. */
	Dictionary      affix_table;
	Afdict_class *  afdict_class;

	/* Random morphology generator */
	struct anysplit_params * anysplit;

	/* If not null, then use spelling guesser for unknown words */
	void *          spell_checker;     /* spell checker handle */
#if USE_CORPUS
	Corpus *        corpus;            /* Statistics database */
#endif
#ifdef HAVE_SQLITE
	void *          db_handle;         /* database handle */
#endif

	void (*insert_entry)(Dictionary, Dict_node *, int);
	Dict_node* (*lookup_list)(Dictionary, const char*);
	Dict_node* (*lookup_wild)(Dictionary, const char*);
	void (*free_lookup)(Dictionary, Dict_node*);
	bool (*lookup)(Dictionary, const char*);
	void (*close)(Dictionary);

	pp_knowledge  * base_knowledge;    /* Core post-processing rules */
	pp_knowledge  * hpsg_knowledge;    /* Head-Phrase Structure rules */
	Connector_set * unlimited_connector_set; /* NULL=everything is unlimited */
	String_set *    string_set;        /* Set of link names in the dictionary */
	Word_file *     word_file_header;

	/* exp_list links together all the Exp structs that are allocated
	 * in reading this dictionary.  Needed for freeing the dictionary
	 */
	Exp_list        exp_list;

	/* Private data elements that come in play only while the
	 * dictionary is being read, and are not otherwise used.
	 */
	const char    * input;
	const char    * pin;
	bool            recursive_error;
	bool            is_special;
	char            already_got_it;
	int             line_number;
	char            token[MAX_TOKEN_LENGTH];
};
/* The functions here are intended for use by the tokenizer, only,
 * and pretty much no one else. If you are not the tokenizer, you
 * probably don't need these. */

bool find_word_in_dict(const Dictionary dict, const char *);

Exp * Exp_create(Exp_list *);
void add_empty_word(Dictionary const, X_node *);
void free_Exp_list(Exp_list *);

#endif /* _LG_DICT_COMMON_H_ */
