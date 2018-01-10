/*************************************************************************/
/* Copyright (c) 2009 Vikas N. Kumar                                     */
/* Copyright (c) 2009 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifdef HAVE_ASPELL

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <aspell.h>

#include "link-includes.h"
#include "spellcheck.h"

#define ASPELL_LANG_KEY  "lang"
/* FIXME: Move to a definition file (affix file?). */
static const char *spellcheck_lang_mapping[] = {
/* link-grammar language , Aspell language key */
	"en", "en_US",
	"ru", "ru_RU",
	"he", "he_IL",
	"de", "de_DE",
	"lt", "lt_LT",
};

struct linkgrammar_aspell {
	AspellConfig *config;
	AspellSpeller *speller;
};

/**
 * create a new spell-checker for the language 'lang'
 */
void * spellcheck_create(const char * lang)
{
	struct linkgrammar_aspell *aspell = NULL;
	size_t i = 0;
	AspellCanHaveError *spell_err = NULL;

	for (i = 0; i < sizeof(spellcheck_lang_mapping)/sizeof(char *); i += 2)
	{
		if (0 != strcmp(lang, spellcheck_lang_mapping[i])) continue;
		aspell = (struct linkgrammar_aspell *)malloc(sizeof(struct linkgrammar_aspell));
		if (!aspell) {
			prt_error("Error: out of memory. Aspell not used.\n");
			aspell = NULL;
			break;
		}
		aspell->config = NULL;
		aspell->speller = NULL;
		aspell->config = new_aspell_config();
		if (aspell_config_replace(aspell->config, ASPELL_LANG_KEY,
					spellcheck_lang_mapping[i]) == 0) {
			prt_error("Error: failed to set language in aspell: %s\n", lang);
			delete_aspell_config(aspell->config);
			free(aspell);
			aspell = NULL;
			break;
		}
		spell_err = new_aspell_speller(aspell->config);
		if (aspell_error_number(spell_err) != 0) {
			prt_error("Error: Aspell: %s\n", aspell_error_message(spell_err));
			delete_aspell_can_have_error(spell_err);
			delete_aspell_config(aspell->config);
			free(aspell);
			aspell = NULL;
			break;
		}
		aspell->speller = to_aspell_speller(spell_err);
		break;
	}
	return aspell;
}

/**
 * Free memory structures used wiith spell-checker 'chk'
 */
void spellcheck_destroy(void * chk)
{
	struct linkgrammar_aspell *aspell = (struct linkgrammar_aspell *)chk;
	if (aspell) {
		delete_aspell_speller(aspell->speller);
		delete_aspell_config(aspell->config);
		free(aspell);
		aspell = NULL;
	}
}

/**
 * Ask the spell-checker if the spelling looks good.
 * Return true if the spelling is good, else false.
 */
bool spellcheck_test(void * chk, const char * word)
{
	int val = 0;
	struct linkgrammar_aspell *aspell = (struct linkgrammar_aspell *)chk;
	if (aspell && aspell->speller)  {
		/* this can return -1 on failure */
		val = aspell_speller_check(aspell->speller, word, -1);
	}
	return (val == 1);
}

int spellcheck_suggest(void * chk, char ***sug, const char * word)
{
	struct linkgrammar_aspell *aspell = (struct linkgrammar_aspell *)chk;
	if (!sug) {
		prt_error("Error: Aspell. Corrupt pointer.\n");
		return 0;
	}
	if (aspell && aspell->speller) {
		const AspellWordList *list = NULL;
		AspellStringEnumeration *elem = NULL;
		const char *aword = NULL;
		unsigned int size, i;
		char **array = NULL;

		list = aspell_speller_suggest(aspell->speller, word, -1);
		elem = aspell_word_list_elements(list);
		size = aspell_word_list_size(list);
		/* allocate an array of char* for returning back to link-parser
		 */
		array = (char **)malloc(sizeof(char *) * size);
		if (!array) {
			prt_error("Error: Aspell. Out of memory.\n");
			delete_aspell_string_enumeration(elem);
			return 0;
		}
		i = 0;
		while ((aword = aspell_string_enumeration_next(elem)) != NULL) {
			array[i++] = strdup(aword);
		}
		delete_aspell_string_enumeration(elem);
		*sug = array;
		return size;
	}
	return 0;
}

void spellcheck_free_suggest(void *chk, char **sug, int size)
{
	int i = 0;
	for (i = 0; i < size; ++i) {
		free(sug[i]);
		sug[i] = NULL;
	}
	free(sug);
}

#endif /* #ifdef HAVE_ASPELL */
