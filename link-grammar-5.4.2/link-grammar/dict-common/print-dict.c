/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2013, 2014 Linas Vepstas                                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <string.h>

#include "api-structures.h" // for Parse_Options_s  (seems hacky to me)
#include "dict-common.h"
#include "dict-defines.h"
#include "print/print.h"
#include "print/print-util.h"
#include "regex-morph.h"
#include "dict-file/word-file.h"
#include "dict-file/read-dict.h"


/* ======================================================================== */

/* INFIX_NOTATION is always defined; we simply never use the format below. */
/* #if ! defined INFIX_NOTATION */
#if 0
/**
 * print the expression, in prefix-style
 */
void print_expression(Exp * n)
{
	E_list * el;
	int i, icost;

	if (n == NULL)
	{
		printf("NULL expression");
		return;
	}

	icost = (int) (n->cost);
	if (n->type == CONNECTOR_type)
	{
		for (i=0; i<icost; i++) printf("[");
		if (n->multi) printf("@");
		printf("%s%c", n->u.string, n->dir);
		for (i=0; i<icost; i++) printf("]");
		if (icost > 0) printf(" ");
	}
	else
	{
		for (i=0; i<icost; i++) printf("[");
		if (icost == 0) printf("(");
		if (n->type == AND_type) printf("& ");
		if (n->type == OR_type) printf("or ");
		for (el = n->u.l; el != NULL; el = el->next)
		{
			print_expression(el->e);
		}
		for (i=0; i<icost; i++) printf("]");
		if (icost > 0) printf(" ");
		if (icost == 0) printf(") ");
	}
}

#else /* INFIX_NOTATION */

#define COST_FMT "%.3f"
/**
 * print the expression, in infix-style
 */
static dyn_str *print_expression_parens(dyn_str *e,
                                        const Exp * n, int need_parens)
{
	E_list * el;
	int i, icost;
	double dcost;

	if (n == NULL)
	{
		append_string(e, "NULL expression");
		return e;
	}

	icost = (int) (n->cost);
	dcost = n->cost - icost;
	if (dcost > 10E-4)
	{
		dcost = n->cost;
		icost = 1;
	}
	else
	{
		dcost = 0;
	}

	/* print the connector only */
	if (n->type == CONNECTOR_type)
	{
		for (i=0; i<icost; i++) dyn_strcat(e, "[");
		if (n->multi) dyn_strcat(e, "@");
		append_string(e, "%s%c", n->u.string, n->dir);
		for (i=0; i<icost; i++) dyn_strcat(e, "]");
		if (0 != dcost) append_string(e, COST_FMT, dcost);
		return e;
	}

	/* Look for optional, and print only that */
	el = n->u.l;
	if (el == NULL)
	{
		for (i=0; i<icost; i++) dyn_strcat(e, "[");
		append_string(e, "()");
		for (i=0; i<icost; i++) dyn_strcat(e, "]");
		if (0 != dcost) append_string(e, COST_FMT, dcost);
		return e;
	}

	for (i=0; i<icost; i++) dyn_strcat(e, "[");
	if ((n->type == OR_type) &&
	    el && el->e && (NULL == el->e->u.l))
	{
		dyn_strcat(e, "{");
		if (NULL == el->next) dyn_strcat(e, "error-no-next");
		else print_expression_parens(e, el->next->e, false);
		append_string(e, "}");
		return e;
	}

	if ((icost == 0) && need_parens) dyn_strcat(e, "(");

	/* print left side of binary expr */
	print_expression_parens(e, el->e, true);

	/* get a funny "and optional" when its a named expression thing. */
	if ((n->type == AND_type) && (el->next == NULL))
	{
		for (i=0; i<icost; i++) dyn_strcat(e, "]");
		if (0 != dcost) append_string(e, COST_FMT, dcost);
		if ((icost == 0) && need_parens) dyn_strcat(e, ")");
		return e;
	}

	if (n->type == AND_type) dyn_strcat(e, " & ");
	if (n->type == OR_type) dyn_strcat(e, " or ");

	/* print right side of binary expr */
	el = el->next;
	if (el == NULL)
	{
		dyn_strcat(e, "()");
	}
	else
	{
		if (el->e->type == n->type)
		{
			print_expression_parens(e, el->e, false);
		}
		else
		{
			print_expression_parens(e, el->e, true);
		}
		if (el->next != NULL)
		{
			// dyn_strcat(e, "\nERROR! Unexpected list!\n");
			/* The SAT parser just naively joins all X_node expressions
			 * using "or", and this check used to give an error due to that,
			 * preventing a convenient debugging.
			 * Just accept it (but mark it with '!'). */
			if (n->type == AND_type) dyn_strcat(e, " &! ");
			if (n->type == OR_type) dyn_strcat(e, " or! ");
			print_expression_parens(e, el->next->e, true);
		}
	}

	for (i=0; i<icost; i++) dyn_strcat(e, "]");
	if (0 != dcost) append_string(e, COST_FMT, dcost);
	if ((icost == 0) && need_parens) dyn_strcat(e, ")");

	return e;
}

void print_expression(const Exp * n)
{
	dyn_str *e = dyn_str_new();

	char *s = dyn_str_take(print_expression_parens(e, n, false));
	err_msg(lg_Debug, "%s\n", s);
	free(s);
}

char *expression_stringify(const Exp * n)
{
	dyn_str *e = dyn_str_new();

	return dyn_str_take(print_expression_parens(e, n, false));
}
#endif /* INFIX_NOTATION */

/* ======================================================================= */

/**
 * Display the information about the given word.
 * If the word can split, display the information about each part.
 * Note that the splits may be invalid grammatically.
 *
 * Wild-card search is supported; the command-line user can type in !!word* or
 * !!word*.sub and get a list of all words that match up to the wild-card.
 * In this case no split is done.
 */
static char *display_word_split(Dictionary dict,
                               const char * word,
                               Parse_Options opts,
                               char * (*display)(Dictionary, const char *))
{
	Sentence sent;
	struct Parse_Options_s display_word_opts = *opts;
	dyn_str *s = dyn_str_new();

	if ('\0' == word) return NULL; /* avoid trying null strings */

	parse_options_set_spell_guess(&display_word_opts, 0);
	sent = sentence_create(word, dict);
	if (0 == sentence_split(sent, &display_word_opts))
	{
		/* List the splits */
		print_sentence_word_alternatives(s, sent, false, NULL, NULL);
		/* List the disjuncts information. */
		print_sentence_word_alternatives(s, sent, false, display, NULL);
	}
	sentence_delete(sent);

	char *out = dyn_str_take(s);
	if ('\0' != out[0]) return out;
	free(out);
	return NULL; /* no dict entry */
}

/**
 * Count the number of clauses (disjuncts) for the expression e.
 * Should return the number of disjuncts that would be returned
 * by build_disjunct().  This in turn should be equal to the number
 * of clauses built by build_clause().
 *
 * Only one minor cheat here: we are ignoring the cost_cutoff, so
 * this potentially over-counts if the cost_cutoff is set low.
 */
static unsigned int count_clause(Exp *e)
{
	unsigned int cnt = 0;
	E_list * e_list;

	assert(e != NULL, "count_clause called with null parameter");
	if (e->type == AND_type)
	{
		/* multiplicative combinatorial explosion */
		cnt = 1;
		for (e_list = e->u.l; e_list != NULL; e_list = e_list->next)
			cnt *= count_clause(e_list->e);
	}
	else if (e->type == OR_type)
	{
		/* Just additive */
		for (e_list = e->u.l; e_list != NULL; e_list = e_list->next)
			cnt += count_clause(e_list->e);
	}
	else if (e->type == CONNECTOR_type)
	{
		return 1;
	}
	else
	{
		assert(false, "an expression node with no type");
	}

	return cnt;
}

/**
 * Count number of disjuncts given the dict node dn.
 */
static unsigned int count_disjunct_for_dict_node(Dict_node *dn)
{
	return (NULL == dn) ? 0 : count_clause(dn->exp);
}

#define DJ_COL_WIDTH sizeof("                         ")

/**
 * Display the number of disjuncts associated with this dict node
 */
static char *display_counts(const char *word, Dict_node *dn)
{
	dyn_str *s = dyn_str_new();

	append_string(s, "matches:\n");
	for (; dn != NULL; dn = dn->right)
	{
		append_string(s, "    %-*s %8u  disjuncts",
		              display_width(DJ_COL_WIDTH, dn->string), dn->string,
		              count_disjunct_for_dict_node(dn));

		if (dn->file != NULL)
		{
			append_string(s, " <%s>", dn->file->file);
		}
		append_string(s, "\n\n");
	}
	return dyn_str_take(s);
}

/**
 * Display the number of disjuncts associated with this dict node
 */
static char *display_expr(const char *word, Dict_node *dn)
{
	dyn_str *s = dyn_str_new();

	append_string(s, "expressions:\n");
	for (; dn != NULL; dn = dn->right)
	{
		char *expstr = expression_stringify(dn->exp);

		append_string(s, "    %-*s %s",
		              display_width(DJ_COL_WIDTH, dn->string), dn->string,
		              expstr);
		free(expstr);
		append_string(s, "\n\n");
	}
	return dyn_str_take(s);
}

static char *display_word_info(Dictionary dict, const char * word)
{
	const char * regex_name;
	Dict_node *dn_head;

	dn_head = dictionary_lookup_wild(dict, word);
	if (dn_head)
	{
		char *out = display_counts(word, dn_head);
		free_lookup_list(dict, dn_head);
		return out;
	}

	/* Recurse, if it's a regex match */
	regex_name = match_regex(dict->regex_root, word);
	if (regex_name)
	{
		return display_word_info(dict, regex_name);
	}

	return NULL;
}

static char *display_word_expr(Dictionary dict, const char * word)
{
	const char * regex_name;
	Dict_node *dn_head;

	dn_head = dictionary_lookup_wild(dict, word);
	if (dn_head)
	{
		char *out = display_expr(word, dn_head);
		free_lookup_list(dict, dn_head);
		return out;
	}

	/* Recurse, if it's a regex match */
	regex_name = match_regex(dict->regex_root, word);
	if (regex_name)
	{
		return display_word_expr(dict, regex_name);
	}

	return NULL;
}

/**
 *  dict_display_word_info() - display the information about the given word.
 */
char *dict_display_word_info(Dictionary dict, const char * word,
		Parse_Options opts)
{
	return display_word_split(dict, word, opts, display_word_info);
}

/**
 *  dict_display_word_expr() - display the connector info for a given word.
 */
char *dict_display_word_expr(Dictionary dict, const char * word, Parse_Options opts)
{
	return display_word_split(dict, word, opts, display_word_expr);
}
