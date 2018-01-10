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

#include "api-structures.h"
#include "api-types.h"
#include "dict-api.h"
#include "dict-common.h"
#include "dict-defines.h" // For MAX_WORD
#include "error.h"
#include "idiom.h"
#include "string-set.h"

/**
 * Find if a string signifies an idiom.
 * Returns true if the string contains an underbar character.
 * The check of s[0] prevents inclusion of '_'. In that case no check for
 * length=1 is done because it is not going to be a valid idiom anyway.
 *
 * If the underbar character is preceded by a backslash, it is not
 * considered.
 */
bool contains_underbar(const char * s)
{
	if ((s[0] == '_') || (s[0] == '\0')) return false;
	while (*++s != '\0')
	{
		if ((*s == '_') && (s[-1] != '\\')) return true;
	}
	return false;
}

/**
 * Returns false if it is not a correctly formed idiom string.
 * Such a string is correct if it:
 *   () contains no SUBSCRIPT_MARK
 *   () non-empty strings separated by _
 */
static bool is_idiom_string(const char * s)
{
	size_t len;
	const char * t;

	for (t = s; *t != '\0'; t++)
	{
		if (*t == SUBSCRIPT_MARK) return false;
	}

	len = strlen(s);
	if ((s[0] == '_') || (s[len-1] == '_'))
	{
		return false;
	}

	for (t = s; *t != '\0'; t++)
	{
		if ((*t == '_') && (*(t+1) == '_')) return false;
	}
	return true;
}

/**
 * Return true if the string s is a sequence of digits.
 */
static bool is_number(const char *s)
{
	while(*s != '\0') {
		if (!isdigit(*s)) return false;
		s++;
	}
	return true;
}

/**
 * If the string contains a SUBSCIPT_MARK, and ends in ".Ix" where
 * x is a number, return x.  Return -1 if not of this form.
 */
static int numberfy(const char * s)
{
	s = strchr(s, SUBSCRIPT_MARK);
	if (NULL == s) return -1;
	if (*++s != 'I') return -1;
	if (!is_number(++s)) return -1;
	return atoi(s);
}

/**
 * Look for words that end in ".Ix" where x is a number.
 * Return the largest x found.
 */
static int max_postfix_found(Dict_node * d)
{
	int i, j;
	i = 0;
	while(d != NULL) {
		j = numberfy(d->string);
		if (j > i) i = j;
		d = d->right;
	}
	return i;
}

/**
 * build_idiom_word_name() -- return idiomized name of given string.
 *
 * Allocates string space and returns a pointer to it.
 * In this string is placed the idiomized name of the given string s.
 * This is the same as s, but with a postfix of ".Ix", where x is an
 * appropriate number.  x is the minimum number that distinguishes
 * this word from others in the dictionary.
 */
static const char * build_idiom_word_name(Dictionary dict, const char * s)
{
	char buff[2*MAX_WORD];
	size_t bufsz = 2*MAX_WORD;
	char *x;
	int count;

	Dict_node *dn = dictionary_lookup_list(dict, s);
	count = max_postfix_found(dn) + 1;
	free_lookup_list(dict, dn);

	x = buff;
	while((*s != '\0') && (*s != SUBSCRIPT_MARK) && (0 < bufsz))
	{
		*x = *s;
		x++;
		s++;
		bufsz--;
	}
	snprintf(x, bufsz, "%cI%d", SUBSCRIPT_MARK, count);

	return string_set_add(buff, dict->string_set);
}

/**
 * Tear the idiom string apart.
 * Put the parts into a list of Dict_nodes (connected by their right pointers)
 * Sets the string fields of these Dict_nodes pointing to the
 * fragments of the string s.  Later these will be replaced by
 * correct names (with .Ix suffixes).
 * The list is reversed from the way they occur in the string.
 * A pointer to this list is returned.
 */
static Dict_node * make_idiom_Dict_nodes(Dictionary dict, const char * string)
{
	Dict_node * dn, * dn_new;
	char * t, *s, *p;
	bool more;
	unsigned int sz;
	dn = NULL;

	sz = strlen(string)+1;
	p = s = (char *) xalloc(sz);
	strcpy(s, string);

	while (*s != '\0') {
		t = s;
		while ((*s != '\0') && (*s != '_')) s++;
		if (*s == '_') {
			more = true;
			*s = '\0';
		} else {
			more = false;
		}
		dn_new = (Dict_node *) xalloc(sizeof (Dict_node));
		dn_new->right = dn;
		dn = dn_new;
		dn->string = string_set_add(t, dict->string_set);
		dn->file = NULL;
		if (more) s++;
	}

	xfree(p, sz);
	return dn;
}

static char current_name[] = "AAAAAAAA";
#define CN_size (sizeof(current_name)-1)

static void increment_current_name(void)
{
	int i = CN_size-1;

	do
	{
		current_name[i]++;
		if (current_name[i] <= 'Z') return;
		current_name[i] = 'A';
	} while (i-- > 0);
	assert(0, "increment_current_name: Overflow");
}

/**
 * Generate a new connector name obtained from the current_name.
 * allocate string space for it.
 * @return a pointer to connector name.
 */
static const char * generate_id_connector(Dictionary dict)
{
	char buff[2*MAX_WORD];
	unsigned int i;
	char * t;

	for (i=0; current_name[i] == 'A'; i++)
	  ;
	/* i is now the number of characters of current_name to skip */
	t = buff;

	/* All idiom connector names start with the two letters "ID" */
	*t++ = 'I';
	*t++ = 'D';
	for (; i < CN_size; i++ )
	{
		*t++ = current_name[i] ;
	}
	*t++ = '\0';
	return string_set_add(buff, dict->string_set);
}

/**
 * Takes as input a pointer to a Dict_node.
 * The string of this Dict_node is an idiom string.
 * This string is torn apart, and its components are inserted into the
 * dictionary as special idiom words (ending in .I*, where * is a number).
 * The expression of this Dict_node (its node field) has already been
 * read and constructed.  This will be used to construct the special idiom
 * expressions.
 * The given dict node is freed.  The string is also freed.
 */
void insert_idiom(Dictionary dict, Dict_node * dn)
{
	Exp * nc, * no, * n1;
	E_list *ell, *elr;
	const char * s;
	Dict_node * dn_list, * xdn, * start_dn_list;

	no = dn->exp;
	s = dn->string;

	if (!is_idiom_string(s))
	{
		prt_error("Warning: Word \"%s\" on line %d "
		          "is not a correctly formed idiom string.\n"
		          "\tThis word will be ignored\n",
		          s, dict->line_number);

		xfree((char *)dn, sizeof (Dict_node));
		return;
	}

	dn_list = start_dn_list = make_idiom_Dict_nodes(dict, s);

	xfree((char *)dn, sizeof (Dict_node));
	dn = NULL;

	assert(dn_list->right != NULL, "Idiom string with only one connector");

	/* first make the nodes for the base word of the idiom (last word) */
	/* note that the last word of the idiom is first in our list */

	/* ----- this code just sets up the node fields of the dn_list ----*/
	nc = Exp_create(&dict->exp_list);
	nc->u.string = generate_id_connector(dict);
	nc->dir = '-';
	nc->multi = false;
	nc->type = CONNECTOR_type;
	nc->cost = 0;

	n1 = Exp_create(&dict->exp_list);
	n1->u.l = ell = (E_list *) xalloc(sizeof(E_list));
	ell->next = elr = (E_list *) xalloc(sizeof(E_list));
	elr->next = NULL;
	ell->e = nc;
	elr->e = no;
	n1->type = AND_type;
	n1->cost = 0;

	dn_list->exp = n1;

	dn_list = dn_list->right;

	while(dn_list->right != NULL)
	{
		/* generate the expression for a middle idiom word */

		n1 = Exp_create(&dict->exp_list);
		n1->u.string = NULL;
		n1->type = AND_type;
		n1->cost = 0;
		n1->u.l = ell = (E_list *) xalloc(sizeof(E_list));
		ell->next = elr = (E_list *) xalloc(sizeof(E_list));
		elr->next = NULL;

		nc = Exp_create(&dict->exp_list);
		nc->u.string = generate_id_connector(dict);
		nc->dir = '+';
		nc->multi = false;
		nc->type = CONNECTOR_type;
		nc->cost = 0;
		elr->e = nc;

		increment_current_name();

		nc = Exp_create(&dict->exp_list);
		nc->u.string = generate_id_connector(dict);
		nc->dir = '-';
		nc->multi = false;
		nc->type = CONNECTOR_type;
		nc->cost = 0;

		ell->e = nc;

		dn_list->exp = n1;

		dn_list = dn_list->right;
	}
	/* now generate the last one */

	nc = Exp_create(&dict->exp_list);
	nc->u.string = generate_id_connector(dict);
	nc->dir = '+';
	nc->multi = false;
	nc->type = CONNECTOR_type;
	nc->cost = 0;

	dn_list->exp = nc;

	increment_current_name();

	/* ---- end of the code alluded to above ---- */

	/* now its time to insert them into the dictionary */

	dn_list = start_dn_list;

	while (dn_list != NULL)
	{
		xdn = dn_list->right;
		dn_list->left = dn_list->right = NULL;
		dn_list->string = build_idiom_word_name(dict, dn_list->string);
		dict->root = insert_dict(dict, dict->root, dn_list);
		dict->num_entries++;
		dn_list = xdn;
	}
	/* xfree((char *)s, s_length+1); strings are handled by string_set */
}

/**
 * returns true if this is a word ending in ".Ix", where x is a number.
 */
bool is_idiom_word(const char * s)
{
	return (numberfy(s) != -1) ;
}
