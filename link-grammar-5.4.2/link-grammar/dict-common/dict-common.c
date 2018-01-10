/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2008, 2009, 2012-2014 Linas Vepstas <linasvepstas@gmail.com>*/
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include "connectors.h"  // for connector_set_delete
#include "dict-affix.h"
#include "dict-api.h"
#include "dict-common.h"
#include "dict-defines.h"
#include "file-utils.h"
#include "post-process/pp_knowledge.h" // Needed only for pp_close !!??
#include "regex-morph.h"
#include "string-set.h"
#include "tokenize/anysplit.h"
#include "tokenize/spellcheck.h"

#include "dict-sql/read-sql.h"
#include "dict-file/read-dict.h"
#include "dict-file/word-file.h"

/* Stems, by definition, end with ".=x" (when x is usually an empty
 * string, i.e. ".="). The STEMSUBSCR definition in the affix file
 * may include endings with other x values, when x serves as a word
 * subscript, e.g. ".=a".  */
#define STEM_MARK '='

/* ======================================================================== */
/* Affix type finding */

/**
 * Return TRUE if the word seems to be in stem form.
 * Stems are signified by including = sign which is preceded by the subscript
 * mark.  Examples (. represented the subscript mark): word.= word.=[!]
 */
bool is_stem(const char* w)
{
	const char *subscrmark = strchr(w, SUBSCRIPT_MARK);

	if (NULL == subscrmark) return false;
	if (subscrmark == w) return false;
	if (STEM_MARK != subscrmark[1]) return false;
	return true;
}

/* ======================================================================== */

Dictionary dictionary_create_default_lang(void)
{
	Dictionary dictionary = NULL;
	char * lang = get_default_locale(); /* E.g. ll_CC.UTF_8 or ll-CC */

	if (lang && *lang)
	{
		lang[strcspn(lang, "_-")] = '\0';
		dictionary = dictionary_create_lang(lang);
	}
	free(lang);

	/* Fall back to English if no default locale or no matching dict. */
	if (NULL == dictionary)
	{
		dictionary = dictionary_create_lang("en");
	}

	return dictionary;
}

Dictionary dictionary_create_lang(const char * lang)
{
	Dictionary dictionary = NULL;

	object_open(NULL, NULL, NULL); /* Invalidate the directory path cache */

	/* If an sql database exists, try to read that. */
	if (check_db(lang))
	{
		dictionary = dictionary_create_from_db(lang);
	}

	/* Fallback to a plain-text dictionary */
	if (NULL == dictionary)
	{
		dictionary = dictionary_create_from_file(lang);
	}

	return dictionary;
}

const char * dictionary_get_lang(Dictionary dict)
{
	if (!dict) return "";
	return dict->lang;
}

/* ======================================================================== */
/* Dictionary lookup stuff */

/**
 * dictionary_lookup_list() - get list of matching words in the dictionary.
 *
 * Returns a pointer to a list of dict_nodes for matching words in the
 * dictionary.
 *
 * This list is made up of Dict_nodes, linked by their right pointers.
 * The exp, file and string fields are copied from the dictionary.
 *
 * The returned list must be freed with free_lookup_list().
 */
Dict_node * dictionary_lookup_list(const Dictionary dict, const char *s)
{
	return dict->lookup_list(dict, s);
}

Dict_node * dictionary_lookup_wild(const Dictionary dict, const char *s)
{
	return dict->lookup_wild(dict, s);
}

void free_lookup_list(const Dictionary dict, Dict_node *llist)
{
	dict->free_lookup(dict, llist);
}

bool boolean_dictionary_lookup(const Dictionary dict, const char *s)
{
	return dict->lookup(dict, s);
}

/**
 * Return true if word is in dictionary, or if word is matched by
 * regex.
 */
bool find_word_in_dict(const Dictionary dict, const char * word)
{
	const char * regex_name;
	if (boolean_dictionary_lookup (dict, word)) return true;

	regex_name = match_regex(dict->regex_root, word);
	if (NULL == regex_name) return false;

	return boolean_dictionary_lookup(dict, regex_name);
}

/* ======================================================================== */
/* the following functions are for handling deletion */

#ifdef USEFUL_BUT_NOT_CURRENTLY_USED
/**
 * Returns true if it finds a non-idiom dict_node in a file that matches
 * the string s.
 *
 * Also sets parent and to_be_deleted appropriately.
 * Note: this function is used in only one place: delete_dictionary_words()
 * which is, itself, not currently used ...
 */
static bool find_one_non_idiom_node(Dict_node * p, Dict_node * dn,
                                   const char * s,
                                   Dict_node **parent, Dict_node **to_be_deleted)
{
	int m;
	if (dn == NULL) return false;
	m = dict_order_bare(s, dn);
	if (m <= 0) {
		if (find_one_non_idiom_node(dn, dn->left, s, parent, to_be_deleted)) return true;
	}
/*	if ((m == 0) && (!is_idiom_word(dn->string)) && (dn->file != NULL)) { */
	if ((m == 0) && (!is_idiom_word(dn->string))) {
		*to_be_deleted = dn;
		*parent = p;
		return true;
	}
	if (m >= 0) {
		if (find_one_non_idiom_node(dn, dn->right, s, parent, to_be_deleted)) return true;
	}
	return false;
}

static void set_parent_of_node(Dictionary dict,
                               Dict_node *p,
                               Dict_node * del,
                               Dict_node * newnode)
{
	if (p == NULL) {
		dict->root = newnode;
	} else {
		if (p->left == del) {
			p->left = newnode;
		} else if (p->right == del) {
			p->right = newnode;
		} else {
			assert(false, "Dictionary broken?");
		}
	}
}

/**
 * This deletes all the non-idiom words of the dictionary that match
 * the given string.  Returns true if some deleted, false otherwise.
 *
 * XXX Note: this function is not currently used anywhere in the code,
 * but it could be useful for general dictionary editing.
 */
int delete_dictionary_words(Dictionary dict, const char * s)
{
	Dict_node *pred, *pred_parent;
	Dict_node *parent, *to_be_deleted;

	if (!find_one_non_idiom_node(NULL, dict->root, s, &parent, &to_be_deleted)) return false;
	for(;;) {
		/* now parent and to_be_deleted are set */
		if (to_be_deleted->file != NULL) {
			to_be_deleted->file->changed = true;
		}
		if (to_be_deleted->left == NULL) {
			set_parent_of_node(dict, parent, to_be_deleted, to_be_deleted->right);
			free_dict_node(to_be_deleted);
		} else {
			pred_parent = to_be_deleted;
			pred = to_be_deleted->left;
			while(pred->right != NULL) {
				pred_parent = pred;
				pred = pred->right;
			}
			to_be_deleted->string = pred->string;
			to_be_deleted->file = pred->file;
			to_be_deleted->exp = pred->exp;
			set_parent_of_node(dict, pred_parent, pred, pred->left);
			free_dict_node(pred);
		}
		if (!find_one_non_idiom_node(NULL, dict->root, s, &parent, &to_be_deleted)) return true;
	}
}
#endif /* USEFUL_BUT_NOT_CURRENTLY_USED */

/**
 * The following two functions free the Exp s and the
 * E_lists of the dictionary.  Not to be confused with
 * free_E_list in word-utils.c.
 */
static void free_Elist(E_list * l)
{
	E_list * l1;

	for (; l != NULL; l = l1) {
		l1 = l->next;
		xfree(l, sizeof(E_list));
	}
}

static inline void exp_free(Exp * e)
{
	xfree((char *)e, sizeof(Exp));
}

static inline void free_dict_node(Dict_node *dn)
{
	xfree((char *)dn, sizeof(Dict_node));
}

void free_Exp_list(Exp_list * eli)
{
	Exp * e1;
	Exp * e = eli->exp_list;
	for (; e != NULL; e = e1)
	{
		e1 = e->next;
		if (e->type != CONNECTOR_type)
		{
		   free_Elist(e->u.l);
		}
		exp_free(e);
	}
}

static void free_dict_node_recursive(Dict_node * dn)
{
	if (dn == NULL) return;
	free_dict_node_recursive(dn->left);
	free_dict_node_recursive(dn->right);
	free_dict_node(dn);
}

static void free_dictionary(Dictionary dict)
{
	free_dict_node_recursive(dict->root);
	free_Word_file(dict->word_file_header);
	free_Exp_list(&dict->exp_list);
}

static void affix_list_delete(Dictionary dict)
{
	int i;
	Afdict_class * atc;
	for (i=0, atc = dict->afdict_class; i < AFDICT_NUM_ENTRIES; i++, atc++)
	{
		if (atc->string) free(atc->string);
	}
	free(dict->afdict_class);
	dict->afdict_class = NULL;
}

void dictionary_delete(Dictionary dict)
{
	if (!dict) return;

	if (verbosity > 0) {
		prt_error("Info: Freeing dictionary %s\n", dict->name);
	}

#ifdef USE_CORPUS
	lg_corpus_delete(dict->corpus);
#endif

	if (dict->affix_table != NULL) {
		affix_list_delete(dict->affix_table);
		dictionary_delete(dict->affix_table);
	}
	spellcheck_destroy(dict->spell_checker);
	if ((locale_t) 0 != dict->lctype) {
		freelocale(dict->lctype);
	}

	connector_set_delete(dict->unlimited_connector_set);

	if (dict->close) dict->close(dict);

	pp_knowledge_close(dict->base_knowledge);
	pp_knowledge_close(dict->hpsg_knowledge);
	string_set_delete(dict->string_set);
	free_regexs(dict->regex_root);
	free_anysplit(dict);
	free_dictionary(dict);
	xfree(dict, sizeof(struct Dictionary_s));
	object_open(NULL, NULL, NULL); /* Free the directory path cache */
}

/* ======================================================================== */
