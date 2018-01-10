/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2008, 2009, 2012 Linas Vepstas <linasvepstas@gmail.com>     */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include "api-structures.h"
#include "dict-common/dict-affix.h"
#include "dict-common/dict-api.h"
#include "dict-common/dict-common.h"
#include "dict-common/dict-defines.h"  // For LEFT_WORD
#include "dict-common/dict-impl.h"
#include "dict-common/dict-utils.h"
#include "dict-common/file-utils.h"
#include "dict-common/idiom.h"
#include "post-process/pp_knowledge.h"
#include "read-dict.h"
#include "read-regex.h"
#include "dict-common/regex-morph.h"
#include "tokenize/anysplit.h"        // Initialize anysplit here ...
#include "tokenize/spellcheck.h"      // Initialize spellcheck here ...
#include "string-set.h"
#include "dict-sql/read-sql.h"  /* Temporary hack */


/***************************************************************
*
* Routines for manipulating Dictionary
*
****************************************************************/

/* Units will typically have a ".u" at the end. Get
 * rid of it, as otherwise stripping is messed up. */
static inline char * deinflect(const char * str)
{
	size_t len;
	char *s;
	char *p = strrchr(str, SUBSCRIPT_MARK);
	if (!p || (p == str)) return strdup(str);

	len = p - str;
	s = (char *)malloc(len + 1);
	strncpy(s, str, len);
	s[len] = '\0';
	return s;
}

static void load_affix(Dictionary afdict, Dict_node *dn, int l)
{
	Dict_node * dnx = NULL;
	for (; NULL != dn; dn = dnx)
	{
		char *string;
		const char *con = word_only_connector(dn);
		if (NULL == con)
		{
			/* ??? should we support here more than one class? */
			prt_error("Warning: Word \"%s\" found near line %d of %s.\n"
			          "\tWord has more than one connector.\n"
			          "\tThis word will be ignored.\n",
			          dn->string, afdict->line_number, afdict->name);
			return;
		}

		/* The affix files serve a dual purpose: they indicate both
		 * what a unit is, connector-wise, and what is strippable, as
		 * a string.  When the unit is an 'idiom' (i.e. two words,
		 * e.g. base_pair or degrees_C) then only the first word can
		 * be stripped away from a run-on expression (e.g. "86degrees C")
		 */
		if (contains_underbar(dn->string))
		{
			char *p;
			string = strdup(dn->string);
			p = string+1;
			while (*p != '_' && *p != '\0') p++;
			*p = '\0';
		}
		else
		{
			string = deinflect(dn->string);
		}

		affix_list_add(afdict, afdict_find(afdict, con,
		               /*notify_err*/true), string);
		free(string);

		dnx = dn->left;
		xfree((char *)dn, sizeof(Dict_node));
	}
}

static void free_llist(Dictionary dict, Dict_node *llist)
{
	file_free_lookup(llist);
}

/**
 * Dummy lookup function for the affix dictionary -
 * compile_regexs() needs it.
 */
static bool return_true(Dictionary dict, const char *name)
{
	return true;
}

static Dictionary
dictionary_six(const char * lang, const char * dict_name,
               const char * pp_name, const char * cons_name,
               const char * affix_name, const char * regex_name);
/**
 * Read dictionary entries from a wide-character string "input".
 * All other parts are read from files.
 */
#define D_DICT 10
static Dictionary
dictionary_six_str(const char * lang,
                   const char * input,
                   const char * dict_name,
                   const char * pp_name, const char * cons_name,
                   const char * affix_name, const char * regex_name)
{
	const char * t;
	Dictionary dict;

	dict = (Dictionary) xalloc(sizeof(struct Dictionary_s));
	memset(dict, 0, sizeof(struct Dictionary_s));

	/* Language and file-name stuff */
	dict->string_set = string_set_create();
	t = strrchr (lang, '/');
	t = (NULL == t) ? lang : t+1;
	dict->lang = string_set_add(t, dict->string_set);
	lgdebug(D_USER_FILES, "Debug: Language: %s\n", dict->lang);
	dict->name = string_set_add(dict_name, dict->string_set);

	/*
	 * A special setup per dictionary type. The check here assumes the affix
	 * dictionary name contains "affix". FIXME: For not using this
	 * assumption, the dictionary creating stuff needs a rearrangement.
	 */
	if (0 == strstr(dict->name, "affix"))
	{
		/* To disable spell-checking, just set the checker to NULL */
		dict->spell_checker = spellcheck_create(dict->lang);
#if defined HAVE_HUNSPELL || defined HAVE_ASPELL
		/* FIXME: Move to spellcheck-*.c */
		if (verbosity_level(D_USER_BASIC) && (NULL == dict->spell_checker))
			prt_error("Info: %s: Spell checker disabled.\n", dict->lang);
#endif
		dict->insert_entry = insert_list;

		dict->lookup_list = file_lookup_list;
		dict->lookup_wild = file_lookup_wild;
		dict->free_lookup = free_llist;
		dict->lookup = file_boolean_lookup;
	}
	else
	{
		/*
		 * Affix dictionary.
		 */
		afclass_init(dict);
		dict->insert_entry = load_affix;
		dict->lookup = return_true;
	}
	dict->affix_table = NULL;

	/* Read dictionary from the input string. */
	dict->input = input;
	dict->pin = dict->input;
	if (!read_dictionary(dict))
	{
		dict->pin = NULL;
		dict->input = NULL;
		goto failure;
	}
	dict->pin = NULL;
	dict->input = NULL;

	if (NULL == affix_name)
	{
		/*
		 * The affix table is handled alone in this invocation.
		 * Skip the rest of processing!
		 * FIXME: The dictionary creating stuff needs a rearrangement.
		 */
		return dict;
	}

	dictionary_setup_locale(dict);

	dict->affix_table = dictionary_six(lang, affix_name, NULL, NULL, NULL, NULL);
	if (dict->affix_table == NULL)
	{
		prt_error("Error: Could not open affix file %s\n", affix_name);
		goto failure;
	}
	if (! afdict_init(dict))
		goto failure;

	if (! anysplit_init(dict->affix_table))
		goto failure;

	/*
	 * Process the regex file.
	 * We have to compile regexs using the dictionary locale,
	 * so make a temporary locale swap.
	 */
	if (read_regex_file(dict, regex_name)) goto failure;

	const char *locale = setlocale(LC_CTYPE, NULL); /* Save current locale. */
	locale = strdupa(locale); /* setlocale() uses its own memory. */
	setlocale(LC_CTYPE, dict->locale);
	lgdebug(+D_DICT, "Regexs locale \"%s\"\n", setlocale(LC_CTYPE, NULL));

	if (compile_regexs(dict->regex_root, dict))
	{
		locale = setlocale(LC_CTYPE, locale);         /* Restore the locale. */
		goto failure;
	}
	locale = setlocale(LC_CTYPE, locale);            /* Restore the locale. */
	assert(NULL != locale, "Cannot restore program locale");

#ifdef USE_CORPUS
	dict->corpus = lg_corpus_new();
#endif

	dict->base_knowledge  = pp_knowledge_open(pp_name);
	dict->hpsg_knowledge  = pp_knowledge_open(cons_name);

	dictionary_setup_defines(dict);

	// Special-case hack.
	if (0 == strcmp(dict->lang, "any") || NULL != dict->affix_table->anysplit)
		dict->shuffle_linkages = true;

	return dict;

failure:
	dictionary_delete(dict);
	return NULL;
}

/**
 * Use filenames of six different files to put together the dictionary.
 */
static Dictionary
dictionary_six(const char * lang, const char * dict_name,
               const char * pp_name, const char * cons_name,
               const char * affix_name, const char * regex_name)
{
	Dictionary dict;

	char* input = get_file_contents(dict_name);
	if (NULL == input)
	{
		prt_error("Error: Could not open dictionary \"%s\"\n", dict_name);
		return NULL;
	}

	dict = dictionary_six_str(lang, input, dict_name, pp_name,
	                          cons_name, affix_name, regex_name);

	free(input);
	return dict;
}

Dictionary dictionary_create_from_file(const char * lang)
{
	Dictionary dictionary;

	init_memusage();
	if (lang && *lang)
	{
		char * dict_name;
		char * pp_name;
		char * cons_name;
		char * affix_name;
		char * regex_name;

		dict_name = join_path(lang, "4.0.dict");
		pp_name = join_path(lang, "4.0.knowledge");
		cons_name = join_path(lang, "4.0.constituent-knowledge");
		affix_name = join_path(lang, "4.0.affix");
		regex_name = join_path(lang, "4.0.regex");

		dictionary = dictionary_six(lang, dict_name, pp_name, cons_name,
		                            affix_name, regex_name);

		free(regex_name);
		free(affix_name);
		free(cons_name);
		free(pp_name);
		free(dict_name);
	}
	else
	{
		prt_error("Error: No language specified!\n");
		dictionary = NULL;
	}

	return dictionary;
}


/**
 * Use "string" as the input dictionary. All of the other parts,
 * including post-processing, affix table, etc, are NULL.
 * This routine is intended for unit-testing ONLY.
 */
Dictionary dictionary_create_from_utf8(const char * input)
{
	Dictionary dictionary = NULL;
	char * lang;

	init_memusage();

	lang = get_default_locale();
	if (lang && *lang) {
		dictionary = dictionary_six_str(lang, input, "string",
		                                NULL, NULL, NULL, NULL);
		free(lang);
	} else {
		/* Default to en when locales are broken (e.g. WIN32) */
		dictionary = dictionary_six_str("en", input, "string",
		                                NULL, NULL, NULL, NULL);
	}

	return dictionary;
}

