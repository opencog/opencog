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

#include <stdio.h>
#include "pp-structures.h"

typedef struct pp_label_node_s
{
	/* linked list of strings associated with a label in the table */
	const char *str;
	struct pp_label_node_s *next;
} pp_label_node;                 /* next=NULL: end of list */

#define PP_LEXER_MAX_LABELS 512
struct PPLexTable_s
{
	String_set *string_set;
	const char *labels[PP_LEXER_MAX_LABELS];             /* array of labels  */
	pp_label_node *nodes_of_label[PP_LEXER_MAX_LABELS]; /*str. for each label*/
	pp_label_node *last_node_of_label[PP_LEXER_MAX_LABELS];    /* efficiency */
	pp_label_node *current_node_of_active_label;/* state: curr node of label */
	int idx_of_active_label;                    /* read state: current label */
	const char **tokens;
	int extents;
};

PPLexTable *pp_lexer_open             (FILE *f);
void  pp_lexer_close                  (PPLexTable *lt);
int   pp_lexer_set_label              (PPLexTable *lt, const char *label);
int   pp_lexer_count_tokens_of_label  (PPLexTable *lt);
const char *pp_lexer_get_next_token_of_label(PPLexTable *lt);
int   pp_lexer_count_commas_of_label  (PPLexTable *lt);
const char **pp_lexer_get_next_group_of_tokens_of_label(PPLexTable *lt, size_t *n_toks);
