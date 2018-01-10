/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2009, 2012-2014 Linas Vepstas                           */
/* Copyright (c) 2014 Amir Plivatsky                                     */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _WIN32
#include <langinfo.h>
#endif
#include <limits.h>

#include "anysplit.h"
#include "api-structures.h"
#include "dict-common/dict-affix.h"
#include "dict-common/dict-api.h"
#include "dict-common/dict-common.h"
#include "dict-common/dict-defines.h" // for MAX_WORD
#include "dict-common/dict-utils.h"
#include "dict-common/regex-morph.h"
#include "error.h"
#include "externs.h"
#include "print/print.h"
#include "print/print-util.h"
#include "spellcheck.h"
#include "string-set.h"
#include "tokenize.h"
#include "tok-structures.h"
#include "utilities.h"
#include "wordgraph.h"
#include "word-structures.h"

#define MAX_STRIP 10
#define SYNTHETIC_SENTENCE_MARK '>' /* A marking of a synthetic sentence. */
#define D_SW 6                      /* debug level for word splits */
#define D_UN 6                      /* debug level for units/punct */

/* These are no longer in use, but are read from the 4.0.affix file */
/* I've left these here, as an example of what to expect. */
/*static char * strip_left[] = {"(", "$", "``", NULL}; */
/*static char * strip_right[] = {")", "%", ",", ".", ":", ";", "?", "!", "''", "'", "'s", NULL};*/
/* Single-quotes are used for abbreviations, don't mess with them */
/*//const char * qs = "\"\'«»《》【】『』‘’`„“"; */
/*const char* qs = "\"«»《》【】『』`„“"; */

#define ENTITY_MARKER   "<marker-entity>"
#define COMMON_ENTITY_MARKER   "<marker-common-entity>"
#define REPLACEMENT_MARK "~" /* a mark for a replacement word */

/* Dictionary capitalization handling */
#define CAP1st "1stCAP" /* Next word is capitalized */
#define CAPnon "nonCAP" /* Next word the lc version of a capitalized word */


/**
 * is_common_entity - Return true if word is a common noun or adjective
 * Common nouns and adjectives are typically used in corporate entity
 * names -- e.g. "Sun State Bank" -- "sun", "state" and "bank" are all
 * common nouns.
 */
static bool is_common_entity(Dictionary dict, const char * str)
{
	if (word_contains(dict, str, COMMON_ENTITY_MARKER) == 1)
		return true;
	return false;
}

static bool is_entity(Dictionary dict, const char * str)
{
	const char * regex_name;
	if (word_contains(dict, str, ENTITY_MARKER) == 1)
		return true;
	regex_name = match_regex(dict->regex_root, str);
	if (NULL == regex_name) return false;
	return word_contains(dict, regex_name, ENTITY_MARKER);
}

#if defined HAVE_HUNSPELL || defined HAVE_ASPELL
/**
 * Return true if word is a proper name.
 * XXX This is a cheap hack that works only in English, and is
 * broken for German!  We need to replace this with something
 * language-specific.
 *
 * Basically, if word starts with upper-case latter, we assume
 * its a proper name, and that's that.
 */
static bool is_proper_name(const char * word, locale_t dict_locale)
{
	return is_utf8_upper(word, dict_locale);
}

/**
 * Returns true if the word contains digits.
 */
static bool contains_digits(const char * s, locale_t dict_locale)
{
	mbstate_t mbs;
	int nb = 1;
	wchar_t c;

	memset(&mbs, 0, sizeof(mbs));
	while ((*s != 0) && (0 < nb))
	{
		nb = mbrtowc(&c, s, MB_CUR_MAX, &mbs);
		if (nb < 0) return false;
		if (iswdigit_l(c, dict_locale)) return true;
		s += nb;
	}
	return false;
}

#if 0
/**
 * Return true if an alternative has been issued for the given word.
 * If there is an alternative, the previous word points to it.
 * Its unsplit_word is the given word.
 *
 * Return true if an alternative is found, else false.
 * XXX need to check correctness.
 * XXX It seems this function is not needed any more. Remove if so.
 */
static bool word_has_alternative(const Gword *word)
{
	const Gword **n;

	for (n = word->prev[0]->next; *n; n++)
	{
		if ((*n)->unsplit_word == word) return true;
	}
	return false;
}
#endif
#endif /* defined HAVE_HUNSPELL || defined HAVE_ASPELL */

/**
 * Find if a string is equal to a character in afdict class afdict_classnum.
 * The parameter afdict_classnum is one of the following affix classes:
 * AFDICT_BULLETS - anything that can be construed to be a bullet.
 * AFDICT_QUOTES  - anything that can be construed to be a quotation mark.
 *
 * Return TRUE if the character is in afdict_classnum.
 */

/**
 * Search in s the first character from utf-8 string xc.
 */
static char *utf8_str1chr(const char *s, const char *xc)
{
	/* FIXME use strndupa() */
	int len = utf8_charlen(xc);
	char *xc1 = alloca(len+1);
	strncpy(xc1, xc, len);
	xc1[len] = '\0';

	return strstr(s, xc1);
}

static bool in_afdict_class(Dictionary dict, afdict_classnum cn, const char *s)
{
	if (0 == AFCLASS(dict->affix_table, cn)->length) return false;
	const char *classchars = AFCLASS(dict->affix_table, cn)->string[0];

	return NULL != utf8_str1chr(classchars, s);
}

/**
 * Return TRUE if the character is white-space
 */
static bool is_space(wchar_t wc, locale_t dict_locale)
{
	if (iswspace_l(wc, dict_locale)) return true;

	/* 0xc2 0xa0 is U+00A0, c2 a0, NO-BREAK SPACE */
	/* For some reason, iswspace doesn't get this */
	if (0xa0 == wc) return true;

	/* iswspace seems to use somewhat different rules than what we want,
	 * so over-ride special cases in the U+2000 to U+206F range.
	 * Caution: this potentially screws with arabic, and right-to-left
	 * languages.
	 */
/***  later, not now ..
	if (0x2000 <= wc && wc <= 0x200f) return true;
	if (0x2028 <= wc && wc <= 0x202f) return true;
	if (0x205f <= wc && wc <= 0x206f) return true;
***/

	return false;
}

#if 0
/**
 * Returns true if the word can be interpreted as a number.
 * The ":" is included here so we allow "10:30" to be a number.
 * The "." and "," allow numbers in both US and European notation:
 * e.g. American million: 1,000,000.00  Euro million: 1.000.000,00
 * We also allow U+00A0 "no-break space"
 */
static bool is_number(Dictionary dict, const char * s)
{
	mbstate_t mbs;
	int nb = 1;
	wchar_t c;
	if (!is_utf8_digit(s, Dictionary dict)) return false;

	memset(&mbs, 0, sizeof(mbs));
	while ((*s != 0) && (0 < nb))
	{
		nb = mbrtowc(&c, s, MB_CUR_MAX, &mbs);
		if (iswdigit_l(dict, c)) { s += nb; }

		/* U+00A0 no break space */
		else if (0xa0 == c) { s += nb; }

		else if ((*s == '.') || (*s == ',') || (*s == ':')) { s++; }
		else return false;
	}
	return true;
}
#endif

static void gwordqueue_add(const Sentence sent, Gword *const word)
{
	struct word_queue *wq_element = malloc(sizeof(*wq_element));

	assert((NULL == sent->word_queue) == (NULL == sent->word_queue_last));

	if (NULL == sent->word_queue)
		sent->word_queue = wq_element;
	else
		sent->word_queue_last->next = wq_element;

	wq_element->word = word;
	wq_element->next = NULL;
	sent->word_queue_last = wq_element;

}

/**
 * Update the label of the word (for debug).
 * The word label marks which part(s) of the program issued the word.
 * This label accumulated if several parts touch the same word.
 */
static void word_label(Sentence sent, Gword *w, const char *op,
                       const char *label)
{
	const size_t s = (NULL == w->label) ? 0 : strlen(w->label);
	char *new_label = alloca(s + strlen(label) + 1 + 2 + 1); /* len+op+()+NUL */

	if (0 != s)
		strcpy(new_label, w->label);
	else
		new_label[0] = '\0';

	if (NULL == op)
		strcat(new_label, "(");
	else if ('\0' != new_label[0])
		strcat(new_label, op);
	strcat(new_label, label);
	if (NULL == op) strcat(new_label, ")");
	w->label = string_set_add(new_label, sent->string_set);
}

#ifdef CHECK_DUPLICATE_ALTS // Not defined - apparently not a problem by now
#define D_WSAA 9
/**
 * Disallow unsplit_word alternatives with the same subword string path.
 * (I.e. we are talking about preventing Wordgraph paths consisting of the same
 * word strings in the same order, not consisting of same graph nodes.)
 *
 * The first subword of the candidate alternative is checked against the first
 * subword of all the existing alternatives of the unsplit_word (this allows the
 * first alternative unconditionally).
 *
 * Parameters:
 * - unsplit_word: the unsplit_word that shouldn't have redundant paths.
 * - altword0: the first word of the candidate alternative.
 * Return true if the alternative is redundant, false if it is fine.
 *
 * Assumptions and heuristics:
 *
 * - This function is invoked only for subwords that we try to split in every
 *   possible combination, i.e. LR-split. It is not invoked for morpheme
 *   splits, because then an identical first subword may be valid due to
 *   a different way of performing the splits.
 *
 * - If the first subword of the checked candidate alternative is the same as an
 *   already existing alternative, it means the candidate alternative is
 *   redundant. This is because we are going to generate all the subword
 *   combinations for the other subwords, a thing that would generate an
 *   identical Wordgraph sub-path otherwise. So the alternative is disallowed.
 *
 * - The previous step may allow an unknown first subword to not split further
 *   to a known word plus punctuation, a thing that will leave an unwanted
 *   unknown word in the Wordgraph. To prevent this we also check if an existing
 *   first subword is a prefix of the first (unknown) subword of the candidate
 *   alternative. If it is, it means that a future split would be prevented by
 *   the previous step.
 *
 * Examples (the words and alternatives are shown in () for clarity):
 *
 * 1. An input word ('50s,) splits to (' 50s,). An additional split of this
 * input word to ('50s ,) would be prevented if '50s was not a known word, as
 * (') is a prefix of ('50s). The split of ('50s) to (' 50s) is prevented by
 * this function because its first subword matches the first subword of an
 * existing alternative (' 50s,).
 *
 * 2. The input word ("who") gets split to (" who"). Its split to ("who ") is
 * then prevented, as (") is a prefix of ("who) which is unknown.
 *
 * XXX I don't have a mathematical proof of correctness, it just happens to work
 * on the example sentences. [ap]
 *
 * FIXME XXX What if a non-first subword is unknown and cannot split further?
 * For example, for ('50s,) we get an alternative (' 50s ,) in which (50s)
 * cannot split further because another alternative also starts with (50), but
 * (50s) is an unknown word (that doesn't usually cause a trouble because
 * tokens in that alternative don't have a linkage). It will be a good idea to
 * find out exactly how it happens and a way to avoid that.  A fix "by force"
 * may be to explicitly mark unknown-words in separate_word() (as originally
 * designed) and in flatten_wordgraph() ignore alternatives that have
 * so-unmarked unknown words.
 */
static bool word_start_another_alternative(Dictionary dict,
                                           Gword *unsplit_word,
                                           const char *altword0)
{
	Gword **n;

	lgdebug(+D_WSAA, "\n"); /* Terminate a previous partial trace message. */
	lgdebug(+D_WSAA, "Checking %s in alternatives of %zu:%s (prev %zu:%s)\n",
	        altword0, unsplit_word->node_num, unsplit_word->subword,
	        unsplit_word->prev[0]->node_num, unsplit_word->prev[0]->subword);

	for (n = unsplit_word->prev[0]->next; NULL != *n; n++)
	{
		if ((*n)->unsplit_word != unsplit_word) continue;
		lgdebug(D_WSAA, "Comparing alt %s\n\\", (*n)->subword);
		if ((0 == strcmp((*n)->subword, altword0) ||
		    ((0 == strncmp((*n)->subword, altword0, strlen((*n)->subword))) &&
			 !find_word_in_dict(dict, altword0))))
		{
			lgdebug(+D_UN, "Preventing alt starts with %s due to existing %zu:%s\n",
			        altword0, (*n)->node_num, (*n)->subword);
			return true;
		}
	}
	return false;
}
#undef D_WSAA
#endif /* CHECK_DUPLICATE_ALTS */

/**
 * Find if a suffix is of a contraction.
 * XXX This is appropriate for English and maybe for some other languages, and
 * may need a generalization.
 * FIXME? Try to work-around the current need of this functions.
 */
static char const *contraction_char[] = { "'", "’" };

#if 0
static bool is_contraction_suffix(const char *s)
{
	size_t len = strlen(s);

	for (size_t i = 0; i < ARRAY_SIZE(contraction_char); i++)
	{
		size_t cclen = strlen(contraction_char[i]);
		if (len < cclen) continue;
		if (0 == strncmp(s+len-cclen, contraction_char[i], cclen)) return true;
	}

	return false;
}

static bool is_contraction_prefix(const char *s)
{
	for (size_t i = 0; i < ARRAY_SIZE(contraction_char); i++)
	{
		size_t cclen = strlen(contraction_char[i]);
		if (0 == strncmp(s, contraction_char[i], cclen)) return true;
	}
	return false;
}
#endif

static bool is_contraction_word(Dictionary dict, const char *s)
{
	if (dict->affix_table && dict->affix_table->anysplit)
		return false;

	for (size_t i = 0; i < ARRAY_SIZE(contraction_char); i++)
	{
		if (NULL != strstr(s, contraction_char[i])) return true;
	}
	return false;
}

/*
 * Return true iff the given word is an AFDICT_xPUNC.
 *
 * FIXME:
 * We cannot directly find if a word is an AFDICT_xPUNC, because
 * we have no way to mark that in * strip_left()/strip_right()/split_mpunc(),
 * and we don't have a direct search function in the afdict (since it
 * doesn't have an in-memory tree structure).
 */
static bool is_afdict_punc(const Dictionary afdict, const char *word)
{
	if (NULL == afdict) return false;
	int punc_types[] = { AFDICT_RPUNC, AFDICT_MPUNC, AFDICT_LPUNC, 0 };

	for (int affix_punc = 0; punc_types[affix_punc] != 0; affix_punc++)
	{
		const Afdict_class * punc_list = AFCLASS(afdict, punc_types[affix_punc]);
		size_t l_strippable = punc_list->length;
		const char * const * punc = punc_list->string;

		for (size_t i = 0; i < l_strippable; i++)
			if (0 == strcmp(word, punc[i])) return true;
	}

	return false;
}

static bool regex_guess(Dictionary dict, const char *word, Gword *gword)
{
		const char *regex_name = match_regex(dict->regex_root, word);
		if ((NULL != regex_name) && boolean_dictionary_lookup(dict, regex_name))
		{
			gword->status |= WS_REGEX;
			gword->regex_name = regex_name;
			return true;
		}
		return false;
}

/**
 * Perform gword_func() on each gword of the given alternative.
 */
static void for_word_alt(Sentence sent, Gword *altp,
                           void(*gword_func)(Sentence sent, Gword *w, unsigned int),
                           unsigned int arg)
{
	Gword *alternative_id = altp->alternative_id;

	if (NULL == altp) return;

	for (; altp->alternative_id == alternative_id; altp = altp->next[0])
	{
		if (NULL == altp) break; /* Just in case this is a dummy word. */

		gword_func(sent, altp, arg);


		/* The alternative ends on one of these conditions:
		 * 1. A different word alternative_id (checked in the loop conditional).
		 * 2. No next word.
		 * 3. The word has been issued alone as its own alternative
		 *    (In that case its alternative_id may belong to a previous
		 *    longer alternative).
		 */
		if ((NULL == altp->next) || altp->issued_unsplit)
			break; /* Only one token in this alternative. */
	}
}

/**
 * Set WS_INDICT / WS_REGEX if the word is in the dict / regex files.
 * The first one which is found is set.
 */
static void set_word_status(Sentence sent, Gword *w, unsigned int status)
{
	switch (status)
	{
		case WS_INDICT|WS_REGEX:
			if (!(w->status & (WS_INDICT|WS_REGEX)))
			{
				if (boolean_dictionary_lookup(sent->dict, w->subword))
				{
					w->status |= WS_INDICT;
				}
				else
				{
					regex_guess(sent->dict, w->subword, w);
				}
			}
			break;

#if defined HAVE_HUNSPELL || defined HAVE_ASPELL
		case WS_RUNON:
		case WS_SPELL:
		/* Currently used to mark words that are a result of a spelling. */
			if ((w->status & WS_INDICT) &&
			    !boolean_dictionary_lookup(sent->dict, w->subword))
			{
				status &= ~WS_INDICT;
			}
			w->status |= status;
			break;
#endif /* HAVE_HUNSPELL */

		default:
			assert(0, "set_dict_word_status: Invalid status 0x%x\n", status);
	}

	lgdebug(+D_SW, "Word %s: status=%s\n", w->subword, gword_status(sent, w));
}

static void set_tokenization_step(Sentence sent, Gword *w, unsigned int ts)
{
	set_word_status(sent, w, WS_INDICT|WS_REGEX);
	w->tokenizing_step = ts;

	lgdebug(+D_SW, "Word %s: status=%s tokenizing_step=%d\n",
			  w->subword, gword_status(sent, w), w->tokenizing_step);
}

/**
 * Prevent a further tokenization.
 * To be used on terminal alternatives.
 */
void tokenization_done(Sentence sent, Gword *altp)
{
	for_word_alt(sent, altp, set_tokenization_step, TS_DONE);
}

/**
 * Issue candidate subwords for unsplit_word (an "alternative").
 * Issue prefnum elements from prefix, stemnum elements from stem, and suffnum
 * elements from suffix.  Mark the prefixes and suffixes with INFIX_MARK (the
 * stems are assumed to be already marked with one of the STEMSUBSCR
 * possibilities.  Set the Morpheme_type of the subwords.
 *
 * The label is used in the wordgraph display, to indicate which section of
 * tokenizing code has inserted the token. If its first character is
 * REPLACEMENT_MARK, the token is not necessarily a substring of the word. This
 * may happen with spell corrections and with the experimental "dictcap"
 * feature, and is used for setting the word position.
 *
 * Return a pointer to the first word of the added alternative.
 *
 * TODO Support also middle morphemes if needed.
 */
#define D_IWA 6
Gword *issue_word_alternative(Sentence sent, Gword *unsplit_word,
                              const char *label,
                              int prefnum, const char * const *prefix,
                              int stemnum, const char * const *stem,
                              int suffnum, const char * const *suffix)
{
	int ai = 0;                     /* affix index */
	const char * const *affix;      /* affix list pointer */
	const char * const * const affixlist[] = { prefix, stem, suffix, NULL };
	const int numlist[] = { prefnum, stemnum, suffnum };
	enum affixtype { PREFIX, STEM, SUFFIX, END };
	enum affixtype at;
	const char infix_mark = INFIX_MARK(sent->dict->affix_table);
	Gword *subword;                 /* subword of the current token */
	Gword *psubword = NULL;         /* subword of the previous token */
	const int token_tot = prefnum + stemnum + suffnum; /* number of tokens */
	Morpheme_type morpheme_type;
	Gword *alternative_id = NULL;   /* to be set to the start subword */
	bool subword_eq_unsplit_word;
	bool last_split = false;        /* this is a final token */
	int *strlen_cache = alloca(token_tot * sizeof(int)); /* token length cache */
#ifdef DEBUG
	Gword *sole_alternative_of_itself = NULL;
#endif

	if (unsplit_word->split_counter > MAX_SPLITS)
	{
		prt_error("Error: Word %s reached %d splits. "
		          "It will not get split further. The result is undefined.\n"
		          "Run with !verbosity="STRINGIFY(D_SW)" to debug\n",
		          unsplit_word->subword, MAX_SPLITS);
		unsplit_word->tokenizing_step = TS_DONE;
		return NULL;
	}
	/* The incremented split_counter will be assigned to the created subwords. */

	lgdebug(+D_IWA, "(%s) Gword %zu:%s split (split_counter=%zu) into", label,
	        unsplit_word->node_num, unsplit_word->subword,
	        unsplit_word->split_counter);

	/* Allocate memory which is enough for the longest token. */
	int maxword = 0;
	for (ai = 0, at = PREFIX; at < END; at++)
	{
		int affixnum = numlist[at];
		char morpheme_sym[] = "pts";

		/* This loop computes too things:
		 * 1. strlen_cache - Token lengths - up to a SUBSCRIPT_MARK if exists.
		 * 2. maxword      - Maximum such token length.  */
		const char subscript_mark_string[] = { SUBSCRIPT_MARK, '\0' };
		for (affix = affixlist[at]; affixnum-- > 0; affix++, ai++)
		{
			strlen_cache[ai] = (int)strcspn(*affix, subscript_mark_string);
			//printf("'%s' strlen_cache[%d]=%d\n",*affix,ai,strlen_cache[ai]);
			maxword = MAX(maxword, strlen_cache[ai]);
			lgdebug(D_IWA, " %c:%s", morpheme_sym[at],
			        ('\0' == (*affix)[0]) ? "[null]" : *affix);
		}
	}

	char * const buff = alloca(maxword + 2); /* strlen + INFIX_MARK + NUL */
	const char *token;

	for (ai = 0, at = PREFIX; at < END; at++)
	{
		int affixnum = numlist[at];

		for (affix = affixlist[at]; affixnum-- > 0; affix++, ai++)
		{
			token = *affix; /* avoid copying if possible */
			switch (at)
			{
				/* Mark the token with INFIX_MARK if needed. */
				case PREFIX: /* set to word= */
					if ('\0' != infix_mark)
					{
						size_t sz = strlen_cache[ai];
						memcpy(buff, *affix, sz);
						buff[sz] = infix_mark;
						buff[sz+1] = '\0';
						last_split = true;
						token = buff;
					}
					if (is_contraction_word(sent->dict, unsplit_word->subword))
						morpheme_type = MT_CONTR;
					else
						morpheme_type = MT_PREFIX;
					break;
				case STEM:   /* already word, word.=, word.=x */
					/* Stems are already marked with a stem subscript, if needed.
					 * The possible marks are set in the affix class STEMSUBSCR. */
					if (is_stem(token))
					{
						morpheme_type = MT_STEM;
						last_split = true;
					}
					else if (is_afdict_punc(sent->dict->affix_table, token))
					{
						morpheme_type = MT_PUNC;
					}
					else
					{
						morpheme_type = MT_WORD;
					}
					break;
				case SUFFIX: /* set to =word */
					/* XXX If the suffix starts with an apostrophe, don't mark it.
					 * Actually - any non-alpha is checked. The random-splitting
					 * "languages" always need the suffix marking. */
					if (((NULL == sent->dict->affix_table->anysplit) &&
					     ('\0' != (*affix)[0]) &&
					     !is_utf8_alpha(*affix, sent->dict->lctype)) ||
					    '\0' == infix_mark)
					{
						if (is_contraction_word(sent->dict, unsplit_word->subword))
							morpheme_type = MT_CONTR;
						else
							morpheme_type = MT_WORD;
						break;
					}
					last_split = true;
					buff[0] = infix_mark;
					strcpy(&buff[1], *affix);
					morpheme_type = MT_SUFFIX;
					token = buff;
					break;
				case END:
					assert(true, "affixtype END reached");
			}

#ifdef CHECK_DUPLICATE_ALTS
			/* FIXME Use another method instead of checking the label. */
			if ((0 == ai) && (1 < token_tot) && (label[0] == 'r') &&
				 word_start_another_alternative(sent->dict, unsplit_word, token))
			{
				/* When called due to left/right strip, the code shouldn't use the
				 * returned value due to the possibility of this returned NULL. */
				return NULL;
			}
#endif /* CHECK_DUPLICATE_ALTS */

			subword_eq_unsplit_word= (0 == strcmp(unsplit_word->subword, token));

			if ((1 == token_tot) && subword_eq_unsplit_word)
			{
				/* Prevent adding a subword as a sole alternative to itself. */
				Gword **q;

				unsplit_word->issued_unsplit = true;

				/*
				 *  If WS_HASALT is unset, then this is the first alternative.
				 */
				if (!(unsplit_word->status & WS_HASALT))
				{
					/* The unsplit_word itself got issued here as the first
					 * alternative of itself. In order that it will not become the
					 * sole alternative of itself, just return. In
					 * remqueue_gword(), issue_word_alternative() is invoked
					 * again if needed - see the next comment. */
					word_label(sent, unsplit_word, "+", label);
					word_label(sent, unsplit_word, NULL, "IU");
					lgdebug(D_IWA, " (issued_unsplit)\n");
					/* Note: The original morpheme_type is preserved.
					 * The morpheme_type value set above is just ignored. */
					return unsplit_word;
				}

				if (unsplit_word->status & WS_UNSPLIT)
				{
					/* If we are here, there is tokenization logic error in the
					 * program, as the word has been issued as an alternative of
					 * itself an additional time. If we proceed it would mess the
					 * Wordgraph pointers. Just warn (if verbosity>0) and return.
					 * The return value is not likely to be used in such a case,
					 * since this is an issuing of a single word.
					 *
					 * Note: In case a new tokenization logic permits adding a
					 * word more than once, just remove this warning. */
					if (0 < verbosity)
					{
						prt_error("Warning: Internal error: "
						          "word \"%s\" got issued more than once\n",
						          unsplit_word->subword);
					}
					return NULL;
				}

				/* We arrive when a word is issued as an alternative of itself and
				 * it already has at least one another alternative. This may happen
				 * when the word is issued as a second and on alternative, or when
				 * we are invoked from remqueue_gword() if it finds that
				 * unsplit_word->issued_unsplit is true and there are
				 * alternatives. Due to the alternatives, the unsplit_word is not
				 * connected to the word flow. We reconnect it here to its former
				 * prev/next words so it will serve as an alternative too. */

				/* Scan its "prev" words and add it as their "next" word */
				for (q = unsplit_word->prev; *q; q++)
					gwordlist_append(&(*q)->next, unsplit_word);
				/* Scan its "next" words and add it as their "prev" word */
				for (q = unsplit_word->next; *q; q++)
					gwordlist_append(&(*q)->prev, unsplit_word);
				word_label(sent, unsplit_word, "+", label);
				word_label(sent, unsplit_word, NULL, "R");
				unsplit_word->status |= WS_UNSPLIT;

				alternative_id = unsplit_word->alternative_id;
#ifdef DEBUG
				sole_alternative_of_itself = unsplit_word;
#endif
				lgdebug(D_IWA, " (reconnected)");

			}
			else
			{
				/* Add the token as a subword of this alternative */
				subword = gword_new(sent, token);
				subword->unsplit_word = unsplit_word;
				subword->split_counter = unsplit_word->split_counter + 1;
				subword->morpheme_type = morpheme_type;
				if (MT_PUNC == morpheme_type) /* It's a terminal token */
					tokenization_done(sent, subword);

				if (last_split)
				{
#if 0
					/* XXX the new Turkish experimental dictionary depend on
					 * specifying compound suffixes which are not in the dict file,
					 * in the SUF affix class. This allows them to split farther.
					 * However, there is a need to detail all the supported
					 * combinations of compound suffixes.
					 * FIXME: There is a need for a real multi affix splitter.
					 * (last_split will get optimized out by the compiler.) */

					/* This is a stem, or an affix which is marked by INFIX_MARK.
					 * Hence it must be a dict word - regex/spell are not done
					 * for stems/affixes. Also, it cannot split further.
					 * Save resources by marking it accordingly. */
					subword->status |= WS_INDICT;
					subword->tokenizing_step = TS_DONE;
#endif
				}
				word_label(sent, subword, "+", label);

				/* If the subword is equal to the unsplit_word (may happen when the
				 * word is issued together with "virtual" morphemes) we should not
				 * queue it for further processing, in order to prevent an infinite
				 * loop. */
				if (!subword_eq_unsplit_word)
					gwordqueue_add(sent, subword);

				/* The spelling properties are inherited over morpheme split */
				if (unsplit_word->status & (WS_SPELL|WS_RUNON))
				    subword->status |= unsplit_word->status & (WS_SPELL|WS_RUNON);

				if (0 == ai) /* first subword of this alternative */
				{
					subword->start = unsplit_word->start;
					if (REPLACEMENT_MARK[0] == label[0])
					{
					/* This is a replacement word (a spell correction or a
					 * "feature" word). Set its end position to the whole
					 * unsplit_word.  For "feature" words this may not be accurate,
					 * but it doesn't matter for now ("dictcap" is experimental). */
						subword->end = unsplit_word->end;
					}
					else
					{
						subword->end = subword->start + strlen_cache[ai];
						/* Account for case conversion length difference. */
						if (subword->status & WS_FIRSTUPPER)
						{
							subword->end +=
								utf8_charlen(unsplit_word->subword) -
								utf8_charlen(token);
						}
						//printf(">>>SUBWORD '%s' %ld:%ld\n", subword->subword, subword->start-sent->orig_sentence, subword->end-sent->orig_sentence);
					}

					if (unsplit_word->status & WS_FIRSTUPPER)
						subword->status |= WS_FIRSTUPPER;

					/* Arrange for subword to be the "next" word of the previous
					 * words of unsplit_word. There are 2 cases:
					 * - If this is not the first alternative - add the subword to
					 *   their "next" links.
					 * - If this is the first alternative - replace the "next" link
					 *   pointing to unsplit_word with a link to subword,
					 *   disconnecting unsplit_word from its RHS. */
					Gword **p;

					alternative_id = subword;

					//previous_wordgraph_nextalts(sent, unsplit_word, subword);
					/* Scan the said previous words. */
					for (p = unsplit_word->prev; NULL != *p; p++)
					{
						Gword **n;

						/* Create the "prev" link for subword */
						gwordlist_append(&subword->prev, *p);

						if (unsplit_word->status & WS_HASALT)
						{
							gwordlist_append(&(*p)->next, subword);
						}
						else
						{
							/* Scan the said "next" links */
							for(n = (*p)->next; NULL != *n; n++)
							{
								if (*n == unsplit_word)
								{
									/* Now finally replace the "next" link */
									*n = subword;
									break;
								}
							}
							assert(NULL != *n, "Adding subword '%s': "
							       "No corresponding next link for a prev link: "
							       "prevword='%s' word='%s'",
							       subword->subword, (*p)->subword, unsplit_word->subword);
						}
					}
				}

				if (token_tot-1 == ai) /* last subword of this alternative */
				{

					/* Arrange for subword to be the "prev" word of the next words of
					 * unsplit_word. There are 2 cases:
					 * - If this is not the first alternative - add the subword to
					 *   their "prev" links.
					 * - If this is the first alternative - replace the "prev" link
					 *   pointing to unsplit_word with a link to subword,
					 *   disconnecting unsplit_word from its LHS.
					 */
					Gword **n;

					//next_wordgraph_prevalts(sent, unsplit_word, subword);
					/* Scan the said next words. */
					for (n = unsplit_word->next; NULL != *n; n++)
					{
						Gword **p;

						/* Create the "next" link for subword */
						gwordlist_append(&subword->next, *n);

						if (unsplit_word->status & WS_HASALT)
						{
							gwordlist_append(&(*n)->prev, subword);
						}
						else
						{
							/* Scan the said "prev" links */
							for(p = (*n)->prev; NULL != *p; p++)
							{
								if (*p == unsplit_word)
								{
									/* Now finally replace the "prev" link */
									*p = subword;
									break;
								}
							}
							assert(NULL!=*p,
								"Adding subword '%s': "
								"No corresponding prev link for a next link"
								"nextword='%s' word='%s'",
								subword->subword, (*n)->subword, unsplit_word->subword);
						}

					}
				}

				if (0 < ai)            /* not the first subword */
				{
					if (REPLACEMENT_MARK[0] == label[0])
					{
						subword->start = unsplit_word->start;
						subword->start = unsplit_word->end;
					}
					else
					{
						subword->start = psubword->end;
						subword->end = subword->start + strlen_cache[ai];
					}

					gwordlist_append(&psubword->next, subword);
					gwordlist_append(&subword->prev, psubword);
				}

				subword->alternative_id = alternative_id;
				psubword = subword;
			}
		}
	}

	unsplit_word->status |= WS_HASALT;
	lgdebug(D_IWA, "\n");

#ifdef DEBUG
			/* Check if the alternative that has just been added already exists.
			 * If it exists - just warn. */
			{
				Gword **prev = unsplit_word->prev;
				Gword *curr_alt = sole_alternative_of_itself ?
					sole_alternative_of_itself : alternative_id;
				Gword **alts;

				assert(curr_alt, "'%s': No alt mark", unsplit_word->subword);
				assert(prev, "'%s': No prev", unsplit_word->subword);
				assert(prev[0], "'%s': No prev[0]", unsplit_word->subword);
				assert(prev[0]->next, "%s': No next",prev[0]->subword);
				assert(prev[0]->next[0], "'%s': No next[0]",prev[0]->subword);
				for (alts = prev[0]->next; *alts; alts++)
				{
					if ((*alts)->unsplit_word != unsplit_word) continue;

					Gword *calt = curr_alt; /* check alternative */
					Gword *oalt; /* old alternatives */
					size_t token_no = token_tot;

					if (*alts == curr_alt) break;
					for (oalt = *alts; token_no > 0; oalt = oalt->next[0])
					{
						if (0 != (strcmp(oalt->subword, calt->subword)))
							break;
						calt = calt->next[0];
						token_no--;
					}
					if (token_tot) continue;
					prt_error("Error: >>>DEBUG>>>: '%s' "
					          "(alternative start '%s', len=%d): "
					          "Alternative already exists!\n",
					          curr_alt->subword, unsplit_word->subword, token_tot);
				}
			}
#endif

	return alternative_id;
}
#undef D_IWA

#define D_RWW 6
static void remqueue_gword(const Sentence sent)
{
	struct word_queue *const wq = sent->word_queue;
	Gword *w = wq->word;

	assert(NULL!=wq, "Trying to remove a word from an empty word queue");

	lgdebug(+D_RWW, "Word '%s'%s%s\n", w->subword,
	        w->issued_unsplit ? " issued_unsplit" : "",
	        w->status & WS_HASALT ? " WS_HASALT" : "");

	/* If the word should have an alternative which includes itself, add it as an
	 * additional alternative (unless it has already been added, as indicated by
	 * WS_UNSPLIT).
	 * See the comments in issue_word_alternative() where remqueue_gword is
	 * mentioned. */
	if (w->issued_unsplit && (w->status & WS_HASALT) && !(w->status & WS_UNSPLIT))
	{
		issue_word_alternative(sent, w, "RQ" ,0,NULL, 1,&w->subword, 0,NULL);
	}

#if WORDGRAPH_PARSER /* not defined */
	/* If the parsers are modified to work directly on the Wordgraph. */
	build_expressions(wq->word);
#endif

	/* Finally, remove the word from the queue. */
	sent->word_queue = wq->next;
	free(wq);
}
#undef D_RWW

static Gword *wordgraph_getqueue_word(Sentence sent)
{
	Gword *w;

	if (NULL == sent->word_queue) return NULL;
	w = sent->word_queue->word;;

	return w;
}

static const char ** resize_alts(const char **arr, size_t len)
{
	arr = realloc(arr, (len+2) * sizeof(char *));
	arr[len+1] = NULL;
	return arr;
}

void altappend(Sentence sent, const char ***altp, const char *w)
{
	size_t n = altlen(*altp);

	*altp = resize_alts(*altp, n);
	(*altp)[n] = string_set_add(w, sent->string_set);
}

/*
	Here's a summary of how subscripts are handled:

	Reading the dictionary:

	  If the last "." in a string is followed by a non-digit character,
	  then the "." and everything after it is considered to be the subscript
	  of the word.

	  The dictionary reader does not allow you to have two words that
	  match according to the criterion below.  (so you can't have
	  "dog.n" and "dog")

	  Quote marks are used to allow you to define words in the dictionary
	  which would otherwise be considered part of the dictionary, as in

	   ";": {@Xca-} & Xx- & (W+ or Qd+) & {Xx+};
	   "%" : (ND- & {DD-} & <noun-sub-x> &
		   (<noun-main-x> or B*x+)) or (ND- & (OD- or AN+));

	Rules for chopping words from the input sentence:

	   First the prefix chars are stripped off of the word.  These
	   characters are "(" and "$" (and now "``")

	   Now, repeat the following as long as necessary:

		   Look up the word in the dictionary.
		   If it's there, the process terminates.

		   If it's not there and it ends in one of the right strippable
		   strings (see "strip_right") then remove the strippable string
		   and make it into a separate word.

		   If there is no strippable string, then the process terminates.

	Rule for defining subscripts in input words:

	   The subscript rule is followed just as when reading the dictionary.

	When does a word in the sentence match a word in the dictionary?

	   Matching is done as follows: Two words with subscripts must match
	   exactly.  If neither has a subscript they must match exactly.  If one
	   does and one doesn't then they must match when the subscript is
	   removed.  Notice that this is symmetric.

	So, under this system, the dictionary could have the words "Ill" and
	also the word "Ill."  It could also have the word "i.e.", which could be
	used in a sentence.
*/

#ifdef DEBUG
/**
 * Split special synthetic words, for Wordgraph handling debug.
 * Word syntax (recursively): LABEL(WORD+WORD+...|WORD+...)
 * Notations in the word syntax:
 *    +: a separator between words of the same alternative.
 *    |: a separator between alternatives.
 *    LABEL: (optional) mark the graph node by a name (for convenience).
 *    (): refers to the unsplit word, in order to generate it as an
 * alternative to itself. E.g. (A|()) generates A as one alternative and the
 * whole unsplit word as the other one.
 * Example sentence: K Ax(BC((mD2+e+F)+(G+h)|(v+w)) C(3|J)) L (()|X+Y)
 * If no split is needed, word syntax errors are silently ignored.
 * Null-string subwords are not allowed, e.g.: A(|B) C(+) D(E|)
 */
static bool synthetic_split(Sentence sent, Gword *unsplit_word)
{
	const char *const w = unsplit_word->subword;
	const char *c = w;
	const char *s = w;
	int plevel = 0;
	const char **alts = NULL;
	bool can_split = false;
	const size_t len = strlen(c);
	char *alt = alloca(len+1);
#define SYNTHSPLIT_ERROR(e) ("Error: synthetic_split(): word '%s':" e "\n")

	/* Synthetic sentences are marked by a special initial character. */
	if (SYNTHETIC_SENTENCE_MARK != sent->orig_sentence[0]) return false;

	assert(0 != len, "synthetic_split(): empty-string word");
	if (')' != w[len-1]) return false; /* no split needed (syntax not checked) */

	do
	{
		switch (*c)
		{
			case '(':
				if (0 == plevel) s = c + 1;
				plevel++;
				break;
			case ')':
			case '+':
			case '|':
				if (1 == plevel)
				{
					if (c == s)
					{
						prt_error(SYNTHSPLIT_ERROR("(empty subword)."), w);
						goto error;
					}
					strncpy(alt, s, c-s);
					alt[c-s] = '\0';
					if (0 == strcmp(alt, "()"))
					{
						/* The word is an alternative to itself. It is not going to
						 * loop due to a special handling in issue_word_alternative().
						 */
						strcpy(alt, w);
					}
					altappend(sent, &alts, alt);
					s = c + 1;

					if ('|' == *c)
					{
						if (alts)
							issue_word_alternative(sent, unsplit_word, "SS", 0,NULL,
							                       altlen(alts),alts, 0,NULL);
						can_split = true;
						free(alts);
						alts = NULL;
					}
				}
				if (')' == *c) plevel--;
				break;
			default:
				if (!(((*c >= 'a') && (*c <= 'z')) ||
				      ((*c >= 'A') && (*c <= 'Z')) ||
				      ((*c >= '0') && (*c <= '9')) ||
				      ('_' == *c)))
				{
					prt_error(SYNTHSPLIT_ERROR("('%c' not alphanumeric)."), w, *c);
					goto error;
				}
		}
		if (0 > plevel)
		{
			prt_error(SYNTHSPLIT_ERROR("extra ')'"), w);
			goto error;
		}

	} while ('\0' != *++c);

	if (0 < plevel)
	{
		prt_error(SYNTHSPLIT_ERROR("missing '('."), w);
		goto error;
	}

	if (alts)
	{
		issue_word_alternative(sent, unsplit_word, "SS", 0,NULL,
		                       altlen(alts),alts, 0,NULL);
		can_split = true;
	}

error:
	free(alts);
	return can_split;
}
#endif

/**
 * Add the given prefix, word and suffix as an alternative.
 * If STEMSUBSCR is define in the affix file, use its values as possible
 * subscripts for the word. In that case, if the word cannot be found in
 * the dict with any of the given stem suffixes, the alternative is not
 * valid and thus not added.
 *
 * If unsplit_word is null, this function actually only checks whether
 * the alternative is valid as described above. This is used for checking
 * is a spell guess result if valid if the word itself is not in the dict.
 * FIXME: If a word can split it doesn't follow it is a "real" dictionary
 * word, as there can still be no links between some of its parts.
 *
 * Return true if the alternative is valid, else false.
 */
static bool add_alternative_with_subscr(Sentence sent,
                                        Gword * unsplit_word,
                                        const char * prefix,
                                        const char * word,
                                        const char * suffix)
{
	Dictionary dict = sent->dict;
	Afdict_class * stemsubscr_list =
	   AFCLASS(dict->affix_table, AFDICT_STEMSUBSCR);
	const char ** stemsubscr = stemsubscr_list->string;
	size_t stemsubscr_count = stemsubscr_list->length;
	bool word_is_in_dict = false;
	bool issue_alternatives = (NULL != unsplit_word);

	if (0 == stemsubscr_count)
	{
		if (issue_alternatives)
		{
			word_is_in_dict = true;
			issue_word_alternative(sent, unsplit_word, "AWS",
			                       (prefix ? 1 : 0),&prefix, 1,&word,
			                       (suffix ? 1 : 0),&suffix);
		}
		else
		{
			/* This is a compound-word spell check. Reject unknown words.
			 * XXX: What if the word is capitalized? */
			word_is_in_dict = boolean_dictionary_lookup(dict, word);
		}
	}
	else
	{
		size_t si;
		size_t wlen = strlen(word);
		size_t slen = 0;
		char *w;

		for (si = 0; si < stemsubscr_count; si++)
		{
			slen = MAX(slen, strlen(stemsubscr[si]));
		}
		w = alloca(wlen + slen + 1);
		strcpy(w, word);

		for (si = 0; si < stemsubscr_count; si++)
		{
			strcpy(&w[wlen], stemsubscr[si]);

			/* We should not match regexes to stems. */
			if (boolean_dictionary_lookup(dict, w))
			{
				word_is_in_dict = true;
				if (issue_alternatives)
				{
					issue_word_alternative(sent, unsplit_word, "AWS",
					   (prefix ? 1 : 0),&prefix, 1,(const char **)&w, 1,&suffix);
				}
			}
		}
	}

	lgdebug(+D_SW,"Stem subscript not found: p:%s t:%s s:%s\n",
	        prefix ? prefix : "(none)", word, suffix ? suffix : "(none)");
	return word_is_in_dict;
}

/**
 * Split word into prefix, stem and suffix.
 * It can also split contracted words (like he's).
 * Alternatives are generated if issue_alternatives=true.
 * Return value:
 *
 * The prefix code is only lightly validated by actual use.
 *
 * If unsplit_word is null, this function actually only checks whether
 * the word can split. This is used for checking if a spell guess result is
 * valid if the word itself is not in the dict. See also
 * add_alternative_with_subscr().
 */
static bool suffix_split(Sentence sent, Gword *unsplit_word, const char *w)
{
	int i, j;
	Afdict_class *prefix_list, *suffix_list;
	int p_strippable, s_strippable;
	const char **prefix, **suffix;
	const char *no_suffix = NULL;
	bool word_can_split = false;
	const Dictionary dict = sent->dict;
	const char *wend = w + strlen(w);
	char *newword = alloca(wend-w+1);

	/* Set up affix tables. */
	if (NULL == dict->affix_table) return false;
	prefix_list = AFCLASS(dict->affix_table, AFDICT_PRE);
	p_strippable = prefix_list->length;
	prefix = prefix_list->string;
	suffix_list = AFCLASS(dict->affix_table, AFDICT_SUF);
	s_strippable = suffix_list->length;
	suffix = suffix_list->string;

	if (INT_MAX == s_strippable) return false;

	/* Go through once for each suffix; then go through one
	 * final time for the no-suffix case (i.e. to look for
	 * prefixes only, without suffixes). */
	for (i = 0; i <= s_strippable; i++, suffix++)
	{
		bool did_split = false;
		size_t suflen = 0;
		if (i < s_strippable)
		{
			suflen = strlen(*suffix);
			 /* The remaining w is too short for a possible match.
			  * In addition, don't allow empty stems. */
			if ((wend-suflen) < (w+1)) continue;

			/* A lang like Russian allows empty suffixes, which have a real
			 * morphological linkage. In the following check, the empty suffix
			 * always matches. */
			if (0 == strncmp(wend-suflen, *suffix, suflen))
			{
				size_t sz = (wend-w)-suflen;
				strncpy(newword, w, sz);
				newword[sz] = '\0';

				/* Check if the remainder is in the dictionary.
				 * In case we try to split a contracted word, the first word
				 * may match a regex. Hence find_word_in_dict() is used and
				 * not boolean_dictionary_lookup().
				 * Note: Not like a previous version, stems cannot match a regex
				 * here, and stem capitalization need to be handled elsewhere. */
				if ((is_contraction_word(dict, w) &&
				    find_word_in_dict(dict, newword)) ||
				    boolean_dictionary_lookup(dict, newword))
				{
					did_split = true;
					word_can_split |=
						add_alternative_with_subscr(sent, unsplit_word,
						                            NULL, newword, *suffix);
				}
			}
		}
		else
		{
			suflen = 0;
			suffix = &no_suffix;
		}

		/*
		 * Try stripping off prefixes. Avoid double-counting and
		 * other trouble by doing this only if we split off a suffix,
		 * or if there is no suffix.
		 */
		if (did_split || 0==suflen)
		{
			for (j = 0; j < p_strippable; j++)
			{
				size_t prelen = strlen(prefix[j]);
				/* The remaining w is too short for a possible match.
				 * NOTE: A zero length "stem" is not allowed here. In any
				 * case, it cannot be handled (yet) by the rest of the code. */
				if ((wend-w) - suflen <= prelen) continue;
				if (strncmp(w, prefix[j], prelen) == 0)
				{
					size_t sz = MIN((wend-w) - suflen - prelen, MAX_WORD);

					strncpy(newword, w+prelen, sz);
					newword[sz] = '\0';
					/* ??? Do we need a regex match? */
					if (boolean_dictionary_lookup(dict, newword))
					{
						word_can_split |=
							add_alternative_with_subscr(sent, unsplit_word, prefix[j],
								                         newword, *suffix);
					}
				}
			}
		}
	}

	return word_can_split;
}

#define HEB_PRENUM_MAX 5   /* no more than 5 prefix "subwords" */
#define HEB_UTF8_BYTES 2   /* Hebrew UTF8 characters are always 2-byte */
#define HEB_CHAREQ(s, c) (strncmp(s, c, HEB_UTF8_BYTES) == 0)
/**
 * Handle "formative letters"  ב, ה, ו, כ, ל, מ, ש.
 * Split word into multiple prefix "subwords" (1-3 characters each)
 * and an unprefixed word (which must be in the dictionary or be null)
 * in all possible ways (even when the prefix combination is not valid,
 * the LG rules will resolve that).
 * If the whole word (i.e. including the prefixes) is in the dictionary,
 * the word will be added in separate_word().
 * Add all the alternatives.
 * The assumptions used prevent a large number of false splits.
 * They may be relaxed later.
 *
 * XXX Because the grammatical rules of which prefixes are valid for the
 * remaining word are not checked, non-existing words may get split. In such a
 * case there is no opportunity for a regex or spell check of this unknown word.
 * FIXME Before issuing an alternative, validate that the combination is
 * supported by the dict.
 *
 * Note: This function currently does more than absolutely needed for LG,
 * in order to simplify the initial Hebrew dictionary.
 * It may be latter replaced by a simpler version.
 *
 * These algorithm is most probably very Hebrew-specific.
 * These assumptions are used:
 * - the prefix consists of subwords
 * - longer subwords have priority over shorter ones
 * - subwords in a prefix are unique ('ככ' is considered here as one "subword")
 * - input words with length <= 2 don't have a prefix
 * - each character uses 2 bytes (can be fixed)
 * - the input word contains only Hebrew characters
 * - the letter "ו" (vav) can only be the first prefix subword
 * - if the last prefix subword is not "ו" and the word (length>2) starts
 *   with 2 "ו", the actual word to be looked up starts with one "ו"
 *   (see also TBD there)
 * - a prefix can be stand-alone (an input word that consists of prefixes)
 *
 * To implement this function in a way which is appropriate for more languages,
 * Hunspell-like definitions (but more general) are needed.
 */
static bool mprefix_split(Sentence sent, Gword *unsplit_word, const char *word)
{
	int i;
	Afdict_class *mprefix_list;
	int mp_strippable;
	const char **mprefix;
	const char *newword;
	const char *w;
	int sz = 0;
	bool word_is_in_dict = false;
	int split_prefix_i = 0;  /* split prefix index */
	const char *split_prefix[HEB_PRENUM_MAX]; /* the whole prefix */
	bool *pseen;             /* prefix "subword" seen (not allowed again) */
	int pfound;              /* index of longer prefix found at a prefix level */
	Dictionary dict = sent->dict;
	int wordlen;
	int wlen;
	int plen = 0;
	Gword *altp;
	bool split_check = (NULL == unsplit_word);

	/* Set up affix table  */
	if (NULL == dict->affix_table) return false;
	mprefix_list = AFCLASS(dict->affix_table, AFDICT_MPRE);
	mp_strippable = mprefix_list->length;
	if (0 == mp_strippable) return false;
	/* The mprefix list is revered-sorted according to prefix length.
	 * The code here depends on that. */
	mprefix = mprefix_list->string;

	pseen = alloca(mp_strippable * sizeof(*pseen));
	/* Assuming zeroed-out bytes are interpreted as false. */
	memset(pseen, 0, mp_strippable * sizeof(*pseen));

	w = word;
	wordlen = strlen(word);  /* guaranteed < MAX_WORD by separate_word() */
	do
	{
		pfound = -1;

		for (i=0; i<mp_strippable; i++)
		{
			/* Subwords in a prefix are unique */
			if (pseen[i])
				continue;

			/* The letter "ו" can only be the first prefix subword */
			if ((split_prefix_i > 0) &&
			 HEB_CHAREQ(mprefix[i], "ו") && (HEB_CHAREQ(w, "ו")))
			{
				continue;
			}

			plen = strlen(mprefix[i]);
			wlen = strlen(w);
			sz = wlen - plen;
			if (strncmp(w, mprefix[i], plen) == 0)
			{
				if (-1 == pfound) pfound = i;
				newword = w + plen;
				/* Check for non-vav before vav */
				if (!HEB_CHAREQ(mprefix[i], "ו") && (HEB_CHAREQ(newword, "ו")))
				{
					/* Non-vav before a single-vav - not in a prefix */
					if (!HEB_CHAREQ(newword+HEB_UTF8_BYTES, "ו"))
						continue;

					/* Non-vav before 2-vav */
					if (newword[HEB_UTF8_BYTES+1])
						newword += HEB_UTF8_BYTES; /* strip one 'ו' */
					/* TBD: check word also without stripping. */
				}
				pseen[i] = true;
				split_prefix[split_prefix_i] = mprefix[i];
				if (0 == sz) /* stand-alone prefix */
				{
					word_is_in_dict = true;
					/* Add the prefix alone */
					lgdebug(+D_UN, "Whole-word prefix: %s\n", word);
					if (split_check) return true;
					altp = issue_word_alternative(sent, unsplit_word, "MPW",
					          split_prefix_i+1,split_prefix, 0,NULL, 0,NULL);
					tokenization_done(sent, altp);
					/* If the prefix is a valid word,
					 * It has been added in separate_word() as a word */
					break;
				}
				if (find_word_in_dict(dict, newword))
				{
					word_is_in_dict = true;
					lgdebug(+D_UN, "Splitting off a prefix: %.*s-%s\n",
					        wordlen-sz, word, newword);
					if (split_check) return true;
					altp = issue_word_alternative(sent, unsplit_word, "MPS",
					          split_prefix_i+1,split_prefix, 1,&newword, 0,NULL);
					tokenization_done(sent, altp);
				}
			}
		}
		if ((-1 != pfound) && (i != pfound))
		{
			/* A previous prefix is the longer one - use it */
			split_prefix[split_prefix_i] = mprefix[pfound];
			plen = strlen(mprefix[pfound]);
			w += plen;
		}
		else
		{
			w = newword;
		}
		split_prefix_i++;
	/* "wlen + sz < wordlen" is true if a vav has been stripped */
	} while ((sz > 0) && (-1 != pfound) && (split_prefix_i < HEB_PRENUM_MAX));

	return word_is_in_dict;
}

/* Return true if the word might be capitalized by convention:
 * -- if its the first word of a sentence
 * -- if its the first word following a colon, a period, a question mark,
 *    or any bullet (For example:  VII. Ancient Rome)
 * -- if its the first word of a quote
 *
 * XXX FIXME: These rules are rather English-centric.  Someone should
 * do something about this someday.
 */
static bool is_capitalizable(const Dictionary dict, const Gword *word)
{
	/* Words at the start of sentences are capitalizable */
	if (MT_WALL == word->prev[0]->morpheme_type) return true;
	if (MT_INFRASTRUCTURE == word->prev[0]->morpheme_type) return true;

	/* Words following colons are capitalizable. */
	/* Mid-text periods and question marks are sentence-splitters. */
	if (strcmp(":", word->prev[0]->subword) == 0 ||
		 strcmp(".", word->prev[0]->subword) == 0 ||
		 strcmp("?", word->prev[0]->subword) == 0 ||
		 strcmp("!", word->prev[0]->subword) == 0 ||
		 strcmp("？", word->prev[0]->subword) == 0 ||
		 strcmp("！", word->prev[0]->subword) == 0 )
		return true;
	if (in_afdict_class(dict, AFDICT_BULLETS, word->prev[0]->subword))
		return true;
	if (in_afdict_class(dict, AFDICT_QUOTES, word->prev[0]->subword))
		return true;

	return false;
}

#define D_MS 6
/*
 * Split the given word "word" to morphemes.
 * If unsplit_word is not NULL then issue alternatives.
 * Else only check the word can split (to validate a spell guess).
 */
static bool morpheme_split(Sentence sent, Gword *unsplit_word, const char *word)
{
	bool word_can_split;

	if (0 < AFCLASS(sent->dict->affix_table, AFDICT_MPRE)->length)
	{
		word_can_split = mprefix_split(sent, unsplit_word, word);
		lgdebug(+D_MS, "Tried mprefix_split word=%s can_split=%d\n",
		        word, word_can_split);
	}
	else
	{
		word_can_split = suffix_split(sent, unsplit_word, word);
		lgdebug(+D_MS, "Tried to split word=%s can_split=%d\n",
		        word, word_can_split);

		/* XXX WS_FIRSTUPPER marking is missing here! */
		if ((NULL != unsplit_word) && is_utf8_upper(word, sent->dict->lctype) &&
		    is_capitalizable(sent->dict, unsplit_word) &&
		    !(unsplit_word->status & (WS_SPELL|WS_RUNON)))
		{
			int downcase_size = strlen(word)+MB_LEN_MAX+1;
			char *const downcase = alloca(downcase_size);

			downcase_utf8_str(downcase, word, downcase_size, sent->dict->lctype);
			word_can_split |=
				suffix_split(sent, unsplit_word, downcase);
			lgdebug(+D_MS, "Tried to split lc=%s now can_split=%d\n",
			        downcase, word_can_split);
		}
	}

	return word_can_split;
}

#if defined HAVE_HUNSPELL || defined HAVE_ASPELL
static bool is_known_word(Sentence sent, const char *word)
{
	return (boolean_dictionary_lookup(sent->dict, word) ||
	        morpheme_split(sent, NULL, word));
}

/**
 * Try to spell guess an unknown word, and issue the results as alternatives.
 * There are two kind of guesses:
 * - Separating run-on words into an exact combination of words, usually 2.
 * - Find similar words. These are limited to use_spell_guess alternatives.
 *
 * Return true if corrections have been issued, else false.
 *
 * Note: spellcheck_suggest(), which is invoked by this function, returns
 * guesses for words containing numbers (including words consisting of digits
 * only). Hence this function should not be called for such words.
 *
 * Note that a lowercase word can be spell-corrected to an uppercase word.
 * FIXME? Should we allow that only if the lc version of the corrected word
 * is the same?
 */
static bool guess_misspelled_word(Sentence sent, Gword *unsplit_word,
                                  Parse_Options opts)
{
	Dictionary dict = sent->dict;
	int runon_word_corrections = 0;
	int num_guesses = 0;
	int j, n;
	char *sp = NULL;
	const char *wp;
	char **alternates = NULL;
	const char *word = unsplit_word->subword;

	/* If the spell-checker knows about this word, and we don't ...
	 * Dang. We should fix it someday. Accept it as such. */
	if (spellcheck_test(dict->spell_checker, word)) return false;

	/* Else, ask the spell-checker for alternate spellings
	 * and see if these are in the dict. */
	n = spellcheck_suggest(dict->spell_checker, &alternates, word);
	if (verbosity_level(+D_SW))
	{
		lgdebug(0, "spellcheck_suggest for %s:\\", word);
		if (0 == n)
			lgdebug(0, " (nothing)\n");
		else
			lgdebug(0, "\n\\");

		for (j=0; j<n; j++)
		{
			if (n-1 != j)
				lgdebug(0, "- %s\n\\", alternates[j]);
			else
				lgdebug(0, "- %s\n", alternates[j]);
		}
	}
	/* Word split for run-on and guessed words.
	 * Note: We suppose here, for the purpose of checking the use_spell_guess
	 * limit, that spellcheck_suggest() returns the run-on splits before the
	 * spell corrections. */
	for (j=0; j<n; j++)
	{
		Gword *altp;

		/* The word might be a run-on of two or more words. */
		sp = strchr(alternates[j], ' ');
		if (sp)
		{
			/* Run-on words.
			 * It may be 2 run-on words or more. Loop over all.
			 */
			const char **runon_word = NULL;
			bool unknown = false;

			wp = alternates[j];
			do
			{
				*sp = '\0';
				unknown |= !is_known_word(sent, wp);
				altappend(sent, &runon_word, wp);
				wp = sp+1;
				sp = strchr(wp, ' ');
			} while (sp);
			unknown |= !is_known_word(sent, wp);
			altappend(sent, &runon_word, wp);
			if (!unknown)
			{
				altp = issue_word_alternative(sent, unsplit_word, "RO",
				          0,NULL, altlen(runon_word),runon_word, 0,NULL);
				for_word_alt(sent, altp, set_word_status, WS_RUNON);
				runon_word_corrections++;
			}
			free(runon_word);
		}
		else
		{
			/* A spell guess.
			 */
			if (is_known_word(sent, alternates[j]))
			{
				wp = alternates[j];
				altp = issue_word_alternative(sent, unsplit_word, REPLACEMENT_MARK "SP",
				                              0,NULL, 1,&wp, 0,NULL);
				for_word_alt(sent, altp, set_word_status, WS_SPELL);
				num_guesses++;
			}
			//else prt_error("Debug: Spell guess '%s' ignored\n", alternates[j]);
		}

		if (num_guesses >= opts->use_spell_guess) break;
	}
	if (alternates) spellcheck_free_suggest(dict->spell_checker, alternates, n);

	return ((num_guesses > 0) || (runon_word_corrections > 0));
}
#endif /* HAVE_HUNSPELL */

static int split_mpunc(Sentence sent, const char *word, char *w,
                                const char *r_stripped[])
{
	const Dictionary afdict = sent->dict->affix_table;
	const Afdict_class * mpunc_list;
	const char * const * mpunc;
	size_t l_strippable;
	int n_r_stripped = 0;

	if (NULL == afdict) return 0;
	mpunc_list = AFCLASS(afdict, AFDICT_MPUNC);
	l_strippable = mpunc_list->length;
	mpunc = mpunc_list->string;

	strcpy(w, word);

	// +1: mpunc in start position is not allowed
	for (char *sep = w+1; '\0' != *sep; sep++)
	{
		for (size_t i = 0; i < l_strippable; i++)
		{
			size_t sz = strlen(mpunc[i]);
			if (0 == strncmp(sep, mpunc[i], sz))
			{
				if ('\0' == sep[sz]) continue; // mpunc in end position

				lgdebug(D_UN, "w='%s' found mpunc '%s'\n", w, mpunc[i]);

				if (sep != w)
				{
					*sep = '\0';
					r_stripped[n_r_stripped++] = w;
				}
				r_stripped[n_r_stripped++] = mpunc[i];

				w = sep + sz;
				sep += sz - 1;
				break;
			}
		}
	}

	if (n_r_stripped > 0) r_stripped[n_r_stripped++] = w;

	if (n_r_stripped >= MAX_STRIP-1)
	{
		lgdebug(+D_SW, "%s %d >= %d tokens\n", __func__, MAX_STRIP, MAX_STRIP-1);
		return 0;
	}

	return n_r_stripped;
}

/**
 * Strip off punctuation, etc. on the left-hand side.
 */
static const char *strip_left(Sentence sent, const char * w,
                       const char *r_stripped[],
                       size_t *n_r_stripped)
{
	const Dictionary afdict = sent->dict->affix_table;
	const Afdict_class * lpunc_list;
	const char * const * lpunc;
	size_t l_strippable;
	size_t i;

	if (NULL == afdict) return (w);
	lpunc_list = AFCLASS(afdict, AFDICT_LPUNC);
	l_strippable = lpunc_list->length;
	lpunc = lpunc_list->string;

	*n_r_stripped = 0;

	do
	{
		for (i=0; i<l_strippable; i++)
		{
			size_t sz = strlen(lpunc[i]);

			if (strncmp(w, lpunc[i], sz) == 0)
			{
				lgdebug(D_UN, "w='%s' found lpunc '%s'\n", w, lpunc[i]);
				r_stripped[(*n_r_stripped)++] = lpunc[i];
				w += sz;
				break;
			}
		}
	/* Note: MAX_STRIP-1, in order to leave room for adding the
	 * remaining word in separate_word(). */
	} while ((i != l_strippable) && (*n_r_stripped < MAX_STRIP-1));

	return (w);
}

/**
 * Split off punctuation and units from the right.
 *
 * Comments from a previous rewrite try of right stripping, not exactly applied
 * to the current method but discuss some relevant problems.
 * ---
 * FIXME: Adapt and move to separate_word().
 * Punctuation and units are removed from the right-hand side of a word,
 * one at a time, until the stem is found in the dictionary; then the
 * stripping stops.  That is, the first dictionary hit wins. This makes
 * sense for punctuation and units, but wouldn't work for morphology
 * in general.
 *
 * The only thing allowed to precede a units suffix is a number. This
 * is so that strings such as "12ft" (twelve feet) are split, but words
 * that accidentally end in "ft" are not split (e.g. "Delft blue")
 * It is enough to ensure that the string starts with a digit.
 *
 * Multiple passes allow for constructions such as 12sq.ft. That is,
 * first, "ft." is stripped, then "sq." is stripped, then "12" is found
 * in the dict.
 *
 * This code is incredibly delicate and fragile. For example, it needs
 * to parse "November 17th, 1771" with the comma stripped, the 17th
 * matched by a regex, but not get a units split for h, i.e. not get
 * the "17t h" split where h==hours-unit.
 *
 * It must also parse "7grams," with the comma split, the s in grams
 * NOT matched by S-WORD regex.
 *
 * Also, 7am must split, even though there's a regex that matches
 * this (the HMS-TIME regex).
 *
 * However, do NOT strip "sin." into "s in." (seconds, inches)
 * or "mold." into "mol d ." (chemistry mol, dioptre)
 * or "call." int "cal l ." (calorie litre)
 * Here, the period means that "sin." is not in the dict.
 *
 * Finally, it must strip the punctuation off of words that are NOT
 * in the dictionary, but will later be split into morphemes, for
 * example, in Lithuanian: "Skaitau knygą."  The period must come off.
 *
 * Basically, don't fuck with this code, unless you run the full
 * set of units sentences in 4.0.fixes.batch.
 *
 * Perhaps someday, improved morphology handling will render this code
 * obsolete. But don't your breath: the interplay between morphology
 * and regex is also quite nasty.
 * ---
 *
 * The classnum parameter is the affix class of strings to strip.
 * The rootdigit parameter adds a check that the root ends with a digit.  It is
 * used when stripping units, so we don't strip units names off potentially real
 * words.
 *
 * w points to the string starting just to the right of any left-stripped
 * characters.
 * n_r_stripped is the index of the r_stripped array, consisting of strings
 * stripped off; r_stripped[0] is the number of the first string stripped off,
 * etc.
 *
 * When it breaks out of this loop, n_r_stripped will be the number of strings
 * stripped off. It is returned through the parameter, after possibly adjusting
 * it so the root will not be null. A pointer to one character after the end of
 * the remaining word is returned through the parameter wend.
 *
 * The function returns true if an affix has been stripped (even if it
 * adjusts n_r_stripped back to 0 if the root was null), else false.
 *
 * p is a mark of the invocation position, for debugging.
 *
 * FIXME: Units may get strip without a root number.
 * In the current batch examples this doesn't cause errors or extraneous
 * parsings. But it unnecessarily adds to the total number of sentence words.
 * For example "sin" is gets split to "s in" and "As" to "A s" (unless an
 * example can be provided in which it is beneficial).
 * The fix should be to reconsider the root number, or remove the code that
 * checks it.
 */
extern const char *const afdict_classname[]; /* For debug message only */
static bool strip_right(Sentence sent,
                        const char *w,
                        const char **wend,
                        const char *r_stripped[],
                        size_t *n_r_stripped,
                        afdict_classnum classnum,
                        bool rootdigit,
                        int p)
{
	Dictionary dict = sent->dict;
	Dictionary afdict = dict->affix_table;
	const char * temp_wend = *wend;
	char *word = alloca(temp_wend-w+1);
	size_t sz;
	size_t i;
	size_t nrs = 0;
	size_t len = 0;
	bool stripped = false;

	Afdict_class *rword_list;
	size_t rword_num;
	const char * const * rword;

	if (*n_r_stripped >= MAX_STRIP-1) return false;

	assert(temp_wend>w, "strip_right: unexpected empty-string word");
	if (NULL == afdict) return false;

	rword_list = AFCLASS(afdict, classnum);
	rword_num = rword_list->length;
	rword = rword_list->string;

	do
	{
		for (i = 0; i < rword_num; i++)
		{
			const char *t = rword[i];

			len = strlen(t);
			/* The remaining w is too short for a possible match */
			if ((temp_wend-w) < (int)len) continue;

			if (strncmp(temp_wend-len, t, len) == 0)
			{
				lgdebug(D_UN, "%d: strip_right(%s): w='%s' rword '%s'\n",
				        p, afdict_classname[classnum], temp_wend-len, t);
				r_stripped[*n_r_stripped+nrs] = t;
				nrs++;
				temp_wend -= len;
				break;
			}
		}
	} while ((i < rword_num) && (temp_wend > w) && rootdigit &&
	         (*n_r_stripped+nrs < MAX_STRIP));
	assert(w <= temp_wend, "A word should never start after its end...");

	sz = temp_wend-w;
	strncpy(word, w, sz);
	word[sz] = '\0';

	/* If there is a non-null root, we require that it ends with a number,
	 * to ensure we stripped off all units. This prevents striping
	 * off "h." from "20th.".
	 * FIXME: is_utf8_digit(temp_wend-1, dict) here can only check ASCII digits,
	 * since it is invoked with the last byte... */
	if (rootdigit && (temp_wend > w) && !is_utf8_digit(temp_wend-1, dict->lctype))
	{
		lgdebug(D_UN, "%d: strip_right(%s): return FALSE; root='%s' (%c is not a digit)\n",
			 p, afdict_classname[classnum], word, temp_wend[-1]);
		return false;
	}

	stripped = nrs > 0;
	if (temp_wend == w)
	{
		/* Null root - undo the last strip */
		nrs--;
		temp_wend += len;
	}

	lgdebug(D_UN, "%d: strip_right(%s): return %s; n_r_stripped=%d+%d, wend='%s' temp_wend='%s'\n",
p, afdict_classname[classnum],stripped?"TRUE":"FALSE",(int)*n_r_stripped,(int)nrs,*wend,temp_wend);

	*n_r_stripped += nrs;
	*wend = temp_wend;
	return stripped;
}

/**
 * Issue an alternative that starts with w and continue with r_stripped[].
 * If wend is NULL, w is Null-terminated.
 */
static void issue_r_stripped(Sentence sent,
                               Gword *unsplit_word,
                               const char *w,
                               const char *wend,
                               const char *r_stripped[],
                               size_t n_r_stripped,
                               const char *nalt)
{
	size_t sz = (NULL==wend) ? strlen(w) : (size_t)(wend-w);
	char *const word = alloca(sz+1);
	const char **rtokens = NULL;
	size_t ntokens = 1;
	int i;

	strncpy(word, w, sz);
	word[sz] = '\0';

	altappend(sent, &rtokens, word);
	lgdebug(+D_SW, "Issue stripped word w='%s' (alt %s)\n", word, nalt);
	for (i = n_r_stripped - 1; i >= 0; i--)
	{
		lgdebug(+D_SW, "Issue r_stripped w='%s' (alt %s)\n", r_stripped[i], nalt);
		altappend(sent, &rtokens, r_stripped[i]);
		ntokens++;
	}
	issue_word_alternative(sent, unsplit_word, nalt,
	                       0,NULL, ntokens,rtokens, 0,NULL);
	free(rtokens);
}

static void issue_dictcap(Sentence sent, bool is_cap,
                          Gword *unsplit_word, const char *word)
{
	const char *dictcap[2];
	Gword *altp;

	dictcap[0] = is_cap ? CAP1st : CAPnon;
	dictcap[1] = word;
	lgdebug(+D_SW, "Adding %s word=%s RE=%s\n", dictcap[0], word,
	        NULL == unsplit_word->regex_name ? "" : unsplit_word->regex_name);
	altp = issue_word_alternative(sent, unsplit_word, REPLACEMENT_MARK "dictcap",
											0,NULL, 2,dictcap, 0,NULL);

	if (NULL == altp)
	{
		prt_error("Warning: Word %s: Internal error: Issuing %s failed\n",
		          dictcap[1], dictcap[0]);
		return;
	}
	/* Set the dictcap[0] word fields */
	altp->status |= WS_INDICT;       /* already checked to be in the dict */
	altp->morpheme_type = MT_FEATURE;
	altp->tokenizing_step = TS_DONE; /* no further tokenization */

	/* Set the alternative word fields. */
	if(is_cap && (NULL != unsplit_word->regex_name))
	{
		/* This is the uc word. */
		altp->next[0]->status |= WS_REGEX;
		altp->next[0]->regex_name = unsplit_word->regex_name;
		/* issue_word_alternative() will mark it as TS_DONE because it appears in
		 * an alternative of itself. */
	}
	else
	{
		/* This is the lc version. The original word can be restored later, if
		 * needed, through the unsplit word. */
		altp->status |= WS_FIRSTUPPER;
	}
}

/* r_stripped debug printout */
static const char *print_rev_word_array(Sentence sent, const char **w,
                                        size_t size)
{
	dyn_str *s = dyn_str_new();
	int i;
	const char *r;

	for (i = size - 1; i >= 0; i--)
		append_string(s, "[%d]='%s'%s", i, w[i], i>0 ? "," : "");

	r = string_set_add(s->str, sent->string_set);
	dyn_str_delete(s);
	return r;
}

/**
 * Check if the word is capitalized according to the regex definitions.
 * XXX Not nice - try to avoid the need of using it.
 */
static bool is_re_capitalized(const char *regex_name)
{
	return ((NULL != regex_name) && (NULL != strstr(regex_name, "CAPITALIZED")));
}

/**
 * Separate a word to subwords in all the possible ways.
 * unsplit_word is the current Wordgraph word to be separated to subwords.
 * This function splits up the word if necessary, and calls
 * "issue_word_alternatives()" on each of the resulting parts ("subwords"),
 * creating an "alternative" to the original unsplit_word.
 *
 * This is used to, e.g, split Russian words into stem+suffix, issuing a
 * separate subword for each.  In addition, there are many English
 * constructions that need splitting:
 *
 * 86mm  -> 86 + mm (millimeters, measurement)
 * $10   ->  $ + 10 (dollar sign plus a number)
 * Surprise!  -> surprise + !  (pry the punctuation off the end of the word)
 * you've   -> you + 've  (undo contraction, treat 've as synonym for 'have')
 *
 * The original separate_word() function directly created the 2D-word-array used
 * by the parser. This version of separate_word() is a rewrite that creates a
 * word graph, referred in the comments as Wordgraph. It is later converted to
 * the said 2D-word-array by flatten_wordgraph().
 *
 * The current separate_word() code is still too similar to the old one, even
 * though some principles of operation are radically different: the separated
 * subwords are now put in a central word queue, from which they are pulled out
 * one by one. If a word is marked by TS_DONE, it will be removed from
 * the word queue without further processing.
 *
 * The function gets each word in the queue, separates it to subwords and create
 * alternatives from each such separation, until all the separating
 * possibilities are exhausted.
 *
 * FIXME: The old code, although working, is convoluted and contains redundant
 * parts. It needs much cleanup efforts, also to make it more flexible and
 * efficient, and at the same time prevent extra splitting (i.e. prevent issuing
 * alternatives which create graph paths with the same sequence of subwords as
 * existing parallel graph paths).
 * A test case: By the '50s, he was very prosperous.
 *
 * XXX This function is being rewritten (work in progress).
 */
static void separate_word(Sentence sent, Gword *unsplit_word, Parse_Options opts)
{
	Dictionary dict = sent->dict;
	bool word_is_known = false;
	bool word_can_split;
	bool word_can_lrmsplit = false;  /* This is needed to prevent spelling on
	                                  * compound subwords, like "Word." while
	                                  * still allowing capitalization handling
	                                  * and regex match. */
	bool lc_word_is_in_dict = false;
	bool stripped;
	const char *wp;
	const char *temp_wend;

	size_t n_r_stripped = 0;
	const char *r_stripped[MAX_STRIP];   /* these were stripped from the right */

	/* For units alternative */
	const char *units_wend = NULL;       /* end of string consisting of units */
	size_t units_n_r_stripped = 0;

	size_t sz = strlen(unsplit_word->subword);
	const char *word = unsplit_word->subword;
	const char *wend = &unsplit_word->subword[sz];

	/* Dynamic allocation of working buffers. */
	int downcase_size = sz+MB_LEN_MAX+1; /* pessimistic max. size of dc buffer */
	char *const downcase = alloca(downcase_size);  /* downcasing buffer */
	char *const temp_word = alloca(downcase_size); /* tmp word buffer */
	char *const seen_word = alloca(downcase_size); /* loop-prevention buffer */

	downcase[0] = '\0';

	lgdebug(+D_SW, "Processing word: '%s'\n", word);

	if (boolean_dictionary_lookup(dict, word))
	{
		lgdebug(+D_SW, "0: Adding '%s' as is, before split tries, status=%s\n",
		        word, gword_status(sent, unsplit_word));
		issue_word_alternative(sent, unsplit_word, "W", 0,NULL, 1,&word, 0,NULL);
		unsplit_word->status |= WS_INDICT;
		word_is_known = true;
	}

	if (unsplit_word->status & (WS_SPELL|WS_RUNON))
	{
		/* The word is a result of spelling, so it doesn't need right/left
		 * stripping. Skip it. */
	}
	else
	{
		if ((MT_CONTR == unsplit_word->morpheme_type))
		{
			/* The word is the contracted part of a contraction. It was most
			 * probably been marked as dict word by the check above (unless there
			 * is a definition error and it is only PRE or SUF without being in the
			 * dict).
			 * It should also not pass any more handling, so return here.
			 * Especially it should not pass right-strip. Else y' gets split to
			 * y ' and 'll gets split as units to ' l l
			 * FIXME This prevents separating double contraction (that still may
			 * not be done even otherwise).
			 * http://en.wiktionary.org/wiki/Category:English_double_contractions*/
			if (!word_is_known)
			{
				/* Note: If we are here it means dict->affix_table is not NULL. */
				prt_error("Warning: Contracted word part %s is in '%s/%s' "
				          "but not in '%s/%s'\n", word,
				          dict->lang, dict->affix_table->name,
				          dict->lang, dict->name);
			}
			return;
		}

		/*
		 * This is essentially the old LR stripping code, from the pre-Wordgraph
		 * version. It still seems to work fine.  Work should be done here in
		 * order to simplify it.
		 */

		wp = strip_left(sent, word, r_stripped, &n_r_stripped);
		if (wp != word)
		{
			/* If n_r_stripped exceed max, the "word" is most likely includes a long
			 * sequence of periods.  Just accept it as an unknown "word",
			 * and move on.
			 * FIXME: Word separation may still be needed, e.g. for a table of
			 * contents:
			 * ............................something
			 * FIXME: "return" here prevents matching a regex.
			 */
			if (n_r_stripped >= MAX_STRIP-1)
			{
				lgdebug(+D_SW, "Left-strip of >= %d tokens\n", MAX_STRIP-1);
				return; /* XXX */
			}

			if ('\0' != *wp)
				r_stripped[n_r_stripped++] = wp;

			issue_word_alternative(sent, unsplit_word, "rL",
			                       0,NULL, n_r_stripped,r_stripped, 0,NULL);

			/* Its possible that the token consisted entirely of
			 * left-punctuation, in which case, wp is an empty-string.
			 * In case this is a single token (n_r_stripped == 1), we have
			 * to continue processing, because it may match a regex. */
			if ('\0' == *wp && n_r_stripped != 1)
			{
				/* Suppose no more alternatives in such a case. */
				lgdebug(+D_SW, "1: Word '%s' all left-puncts - done\n",
						  unsplit_word->subword);
				return;
			}

			n_r_stripped = 0;
			word_can_lrmsplit = true;
		}

		lgdebug(+D_SW, "1: Continue with word %s status=%s\n",
		        word, gword_status(sent, unsplit_word));

		/* Strip off punctuation and units, etc. on the right-hand side.  Try
		 * rpunc, then units, then rpunc, then units again, in a loop. We do this
		 * to handle expressions such as 12sqft. or 12lbs. (notice the period at
		 * end). That is, we want to strip off the "lbs." with the dot, first,
		 * rather than stripping the dot as punctuation, and then coming up
		 * empty-handed for "sq.ft" (without the dot) in the dict.  But if we are
		 * NOT able to strip off any units, then we try punctuation, and then
		 * units. This allows commas to be removed (e.g.  7grams,). */

		seen_word[0] = '\0';
		do
		{
			int temp_n_r_stripped;
			/* First, try to strip off a single punctuation, typically a comma or
			 * period, and see if the resulting word is in the dict (but not the
			 * regex). This allows "sin." and "call." to be recognized. If we don't
			 * do this now, then the next stage will split "sin." into
			 * seconds-inches, and "call." into calories-liters. */
			temp_n_r_stripped = n_r_stripped;
			temp_wend = wend;
			stripped = strip_right(sent, word, &wend, r_stripped, &n_r_stripped,
			                       AFDICT_RPUNC, /*rootdigit*/false, 2);
			if (stripped)
			{
				/* "wend" points to the end of the remaining word. */
				sz = wend-word;
				strncpy(temp_word, word, sz);
				temp_word[sz] = '\0';

				/* If the resulting word is in the dict, we are done. */
				if (boolean_dictionary_lookup(dict, temp_word)) break;
				/* Undo the check. */
				wend = temp_wend;
				n_r_stripped = temp_n_r_stripped;
			}

			/* Remember the results, for a potential alternative. */
			units_wend = wend;
			units_n_r_stripped = n_r_stripped;

			/* Strip off all units, if possible. It is not likely that we strip
			 * here a string like "in." which is not a unit since we require a
			 * number before it when only a single component is stripped off. */
			temp_wend = wend;
			stripped = strip_right(sent, word, &wend, r_stripped, &n_r_stripped,
			                       AFDICT_UNITS, /*rootdigit*/true, 3);
			if (!stripped)
			{
				units_wend = NULL;
				/* Try to strip off punctuation, typically a comma or period. */
				stripped = strip_right(sent, word, &wend, r_stripped, &n_r_stripped,
				                       AFDICT_RPUNC, /*rootdigit*/false, 4);
			}

			/* w points to the remaining word,
			 * "wend" to the end of the word. */
			sz = wend-word;
			strncpy(temp_word, word, sz);
			temp_word[sz] = '\0';

			/* Avoid an infinite loop in case of a repeating unknown remaining word */
			if (0 == strcmp(temp_word, seen_word)) break;
			strcpy(seen_word, temp_word);

		/* Any remaining dict word stops the right-punctuation stripping. */
		} while (NULL == units_wend && stripped &&
					!boolean_dictionary_lookup(dict, temp_word));

		lgdebug(+D_SW, "After strip_right: n_r_stripped=(%s) "
		        "word='%s' wend='%s' units_wend='%s' temp_word='%s'\n",
		        print_rev_word_array(sent, r_stripped, n_r_stripped),
		        word, wend, units_wend, temp_word);

		/* If n_r_stripped exceed max, the "word" most likely includes a long
		 * sequence of periods.  Just accept it as an unknown "word",
		 * and move on.
		 * FIXME: Word separation may still be needed, e.g. for a table of
		 * contents:
		 * 10............................
		 */
		if (n_r_stripped >= MAX_STRIP-1)
		{
			lgdebug(+D_SW, "Right-strip of >= %d tokens\n", MAX_STRIP-1);
			return; /* XXX */
		}

		/* Check whether the <number><units> "word" is in the dict (including
		 * regex). In such a case we need to generate an alternative. This happens
		 * if it is a part number, like "1234-567A".
		 */

		if (units_n_r_stripped && units_wend) /* units found */
		{
			sz = units_wend-word;
			strncpy(temp_word, word, sz);
			temp_word[sz] = '\0';

			if (find_word_in_dict(dict, temp_word))
			{
				issue_r_stripped(sent, unsplit_word, temp_word, NULL,
										 r_stripped, units_n_r_stripped, "rR2");
				word_can_lrmsplit = true;
			}
		}


		/* Add the strip result as an alternative if one of these conditions is
		 * true:
		 * - If the root word (temp_word) is known.
		 * - If the unsplit_word is unknown. This happens with an unknown word
		 *   that has punctuation after it). */
		if (n_r_stripped > 0)
		{
			sz = wend-word;
			strncpy(temp_word, word, sz);
			temp_word[sz] = '\0';

			if (!find_word_in_dict(dict, unsplit_word->subword) ||
			    find_word_in_dict(dict, temp_word))
			{
				issue_r_stripped(sent, unsplit_word, temp_word, NULL,
										 r_stripped, n_r_stripped, "rR3");
				word_can_lrmsplit = true;
			}
		}
	}

	n_r_stripped = split_mpunc(sent, word, temp_word, r_stripped);
	if (n_r_stripped > 0)
	{
		issue_word_alternative(sent, unsplit_word, "M", 0,NULL,
		                       n_r_stripped,r_stripped, 0,NULL);
		word_can_lrmsplit = true;
	}

	lgdebug(+D_SW, "2: Continue with word=%s can_lrmsplit=%d status=%s\n",
	        word, word_can_lrmsplit, gword_status(sent, unsplit_word));

	/* Generate random morphology */
	if ((dict->affix_table && dict->affix_table->anysplit) && !word_can_lrmsplit)
	if (dict->affix_table && dict->affix_table->anysplit)
		anysplit(sent, unsplit_word);

	/* OK, now try to strip affixes. */
	word_can_split = morpheme_split(sent, unsplit_word, word);

	/* If the word is unknown, then try to guess its category by regexes.
	 * A word that cannot split is considered known, unless it is a contraction,
	 * in which case we need a regex for things like 1960's.
	 * The first regex which matches (if any) is used.
	 * An alternative consisting of the word has already been generated. */
	if (!word_is_known && (!word_can_split || is_contraction_word(dict, word)))
	{
		regex_guess(dict, word, unsplit_word);
		/* Even if a regex matches, don't set word_is_known=true yet. */
	}

	lgdebug(+D_SW, "After split step, word=%s can_split=%d is_known=%d RE=%s\n",
	        word, word_can_split, word_is_known,
	        (NULL == unsplit_word->regex_name) ? "" : unsplit_word->regex_name);

	/* FIXME: Handling of capitalized words that are a result of spelling. */
	if (is_utf8_upper(word, dict->lctype))
	{
		if (!test_enabled("dictcap"))
		{
			/** Hard-coded English-centric capitalization handling.
			 *
			 * FIXME: Capitalization handling should be done using the dict.
			 *
			 * If the word is capitalized, then issue as alternatives:
			 * - Issue its lowercase version if it is in a capitalizable
			 *   position and also it is in the dict.
			 * - Issue it (capitalized) too as a word to regex (so the
			 *   capitalized-words regex disjuncts will be used), in these
			 *   conditions (cumulative):
			 *   -- It could not be split (else capitalization has been
			 *      handled XXX).
			 *   -- It is not in the dict (it has already been issued in
			 *      that case).
			 *   -- It is not in a capitalizable position in the sentence.
			 *   -- Its lowercase version is in the dict file (not regex) and
			 *      it is an entity (checked capitalized) or a common entity
			 *      (checked as lowercase).
			 *
			 *   Comments from a previous release:
			 *
			 * * Common entity (checked as lowercase): This allows common
			 *   nouns and adjectives to be used for entity names: e.g. "Great
			 *   Southern Union declares bankruptcy", allowing Great to be
			 *   capitalized, while preventing an upper-case "She" being used
			 *   as a proper name in "She declared bankruptcy".
			 *
			 * * Entity (checked capitalized): We need to *add* Sue.f (female
			 *   name Sue) even though sue.v (the verb "to sue") is in the
			 *   dict.  So test for capitalized entity names.  FIXME: [ap]
			 *   Since capitalized words which are in the dict file are now
			 *   issued anyway as uppercase, and the capitalized-words regexes
			 *   are not marked in the dict as entities, this may have effect
			 *   only for capitalized words that match non-capitalized-words
			 *   regexes that are marked as entities. I don't know about such,
			 *   and if there are indeed no such regexes, it looks like the
			 *   is_entity() check is redundant.  A test "is_entity" added
			 *   below to check if there is any sentence in the batches that
			 *   contradicts that.
			 */
			bool word_is_capitalizable = is_capitalizable(dict, unsplit_word);

			if ('\0' == downcase[0])
				downcase_utf8_str(downcase, word, downcase_size, dict->lctype);
			lc_word_is_in_dict = boolean_dictionary_lookup(dict, downcase);

			if (word_is_capitalizable)
			{
				if (lc_word_is_in_dict)
				{
					/* Issue the lowercase version of the word. */
					Gword *lc;

					wp = downcase;
					lgdebug(+D_SW, "Adding lc=%s is_capitalizable=1\n", wp);
					lc = issue_word_alternative(sent, unsplit_word, "LC",
					                            0,NULL, 1,&wp, 0,NULL);
					if (NULL == lc)
					{
						prt_error("Warning: Word %s: Internal error: Issuing lc failed\n",
									 wp);
						return;
					}
					/* This is the lc version. The original word can be restored
					 * later, if needed, through the unsplit word. */
					lc->status |= WS_FIRSTUPPER;
				}
				else /* for a comment */
				{
					/* If we are here, it is a capitalized word in a capitalized
					 * position which its lowercase version is not in the dict file.
					 * Should we try a regex match if the word is unknown? */
				}
			}

			lgdebug(+D_SW, "Word=%s lc=%s in_dict=%d is_known=%d can_split=%d "
			        "is_capitalizable=%d lc_is_in_dict=%d "
			        "is_entity=%d is_common_entity=%d\n",
			        word, downcase, !!(unsplit_word->status & WS_INDICT),
			        word_is_known, word_can_split,
			        word_is_capitalizable, lc_word_is_in_dict,
			        is_entity(dict, word), is_common_entity(dict, downcase));

			if (!word_can_split && !word_is_known &&
				 (!word_is_capitalizable || (lc_word_is_in_dict &&
					(is_common_entity(dict, downcase) || is_entity(dict, word)))))
			{
				/* Issue it (capitalized) too */
				if ((NULL != unsplit_word->regex_name))
				{
					lgdebug(+D_SW, "Adding uc word=%s RE=%s\n", word,
					        unsplit_word->regex_name);
					issue_word_alternative(sent, unsplit_word, "REuc",
					                       0,NULL, 1,&word, 0,NULL);
					word_is_known = true;

					if (test_enabled("is_entity") && is_entity(dict, word))
						prt_error("is_entity(%s): %s\n", word, sent->orig_sentence);
				}
			}
			word_is_known |= lc_word_is_in_dict;
		}
		else
		{
			/*
			 * Experimental dictionary handling for capitalized words.
			 */

			if (!boolean_dictionary_lookup(dict, CAP1st) ||
				 !boolean_dictionary_lookup(dict, CAPnon))
			{
				/* FIXME Move this check. Make it once. */
				prt_error("Error: Missing " CAP1st "/" CAPnon "in the dict\n");
				return;
			}

			/* - If the (uc) word is in the dict, it has already been issued.
			 * - If the word is not a capitalized word according to the regex file,
			 *   it also should not be issued, even if is_utf8_upper(word, dict),
			 *   e.g Y'gonna or Let's. */
			if (!(unsplit_word->status & WS_INDICT) &&
			    is_re_capitalized(unsplit_word->regex_name))
			{
				issue_dictcap(sent, /*is_cap*/true, unsplit_word, word);
			}

			downcase_utf8_str(downcase, word, downcase_size, dict->lctype);
			/* Issue the lc version if it is known.
			 * FIXME? Issuing only known lc words prevents using the unknown-word
			 * device for words in capitalizable position (when the word is a uc
			 * version of an unknown word). */
			if (find_word_in_dict(sent->dict, downcase))
				issue_dictcap(sent, /*is_cap*/false, unsplit_word, downcase);

			word_is_known = true; /* We could just return */
		}
	}

	/* Handle regex match. This is done for words which are not in the dict
	 * and cannot morpheme split.
	 *
	 * Contracted words, like 1960's should be tried - words that contain
	 * punctuation are not going to match).
	 *
	 * However, capital LR-split words which their lc version is in the dict,
	 * such as "As" (gets split to A s) shouldn't be tried here, as their
	 * capitalization handling has already been handled before we arrived here,
	 * and if a capital-word regex has not been issued there, we should prevent
	 * issuing it here. */
	if (!(word_is_known ||  lc_word_is_in_dict ||
	      (word_can_split && !is_contraction_word(dict, word))))
	{
		if ((NULL != unsplit_word->regex_name))
		{
			lgdebug(+D_SW, "Adding word '%s' for regex, match=%s\n",
			        word, unsplit_word->regex_name);
			issue_word_alternative(sent, unsplit_word, "RE",
			                       0,NULL, 1,&word, 0,NULL);

		   word_is_known = true; /* make sure we skip spell guess */
		}
	}

	word_is_known |= word_can_split;

#if defined HAVE_HUNSPELL || defined HAVE_ASPELL
	/* If the word is unknown, then it might be a run-on of two words or a
	 * misspelled word. Ask the spell-checker to split the word, if possible,
	 * and/or offer guesses.
	 *
	 * Do all of this only if the word is not a proper name, and if
	 * spell-checking is enabled and spell-checker is specified. A word which
	 * contains digits is considered a proper name (maybe of a part number).
	 *
	 * ??? Should we add spell guesses as alternatives in case:
	 * 1. The word if not in the main dict but matches a regex.
	 * 2. The word an unknown capitalized word.
	 */
	if (!word_can_lrmsplit && !word_is_known &&
	    !contains_digits(word, dict->lctype) &&
	    !is_proper_name(word, dict->lctype) &&
	    opts->use_spell_guess && dict->spell_checker)
	{
		bool spell_suggest = guess_misspelled_word(sent, unsplit_word, opts);
		lgdebug(+D_SW, "Spell suggest=%d\n", spell_suggest);
	}
#endif /* defined HAVE_HUNSPELL || defined HAVE_ASPELL */

	lgdebug(+D_SW, "END: Word '%s' in_dict=%d is_known=%d status=%s\n",
	        unsplit_word->subword, !!(unsplit_word->status & WS_INDICT),
	        word_is_known, gword_status(sent, unsplit_word));
#if 0
	if (!word_is_known &&
	    !(unsplit_word->status & (WS_INDICT|WS_REGEX)))
		unsplit_word->status |= WS_UNKNOWN;
#endif
}

/**
 * Make the string 's' be the next word of the sentence.
 *
 * Do not issue the empty string.
 *
 * FIXME: We need it for now in order to initially populate the wordgraph.
 */
static Gword *issue_sentence_word(const Sentence sent, const char *const s)
{
	Gword *new_word;
	Gword *last_word = sent->last_word;

	assert(NULL!=last_word);
	assert(NULL!=s, "subword must not be NULL");
	assert('\0'!=s[0], "subword must not be an empty-string: "
	                   "Last subword issued: '%s'", last_word->subword);

	new_word = gword_new(sent, s);
	new_word->unsplit_word = sent->wordgraph;
	new_word->label = "S"; /* a sentence word */

	gwordlist_append(&last_word->next, new_word);
	gwordlist_append(&new_word->prev, last_word);

	gwordqueue_add(sent, new_word);

	return new_word;
}

static void add_gword(Sentence sent, const char *w, const char *wend,
                               Morpheme_type morpheme_type)
{
	const size_t sz = (NULL == wend) ? strlen(w) : (size_t)(wend - w);
	char *const word = alloca(sz+1);
	Gword *new_word;

	strncpy(word, w, sz);
	word[sz] = '\0';

	new_word = issue_sentence_word(sent, word);
	new_word->morpheme_type = morpheme_type;
	new_word->alternative_id = sent->wordgraph;
	if (NULL != wend)
	{
		new_word->start = w;
		new_word->end = wend;
	}
	if (MT_WORD != morpheme_type)
	{
		/* Skip tokenizing this word */
		new_word->tokenizing_step = TS_DONE;
		if (MT_WALL == morpheme_type)
		{
			new_word->status |= WS_INDICT;
			if (MT_INFRASTRUCTURE == new_word->prev[0]->morpheme_type)
				new_word->start = sent->orig_sentence;
			else
				new_word->start = sent->orig_sentence + strlen(sent->orig_sentence);
			new_word->end = new_word->start;
		}
	}
}

/**
 * Create the first node of the wordgraph.
 * Its subword is the whole original sentence.
 * It also serves as a leading dummy "word" for the sentence, like a "margin
 * mark", in order to simplify the wordgraph pointer manipulation (no need to
 * check for NULL "prev" pointer). The unsplit_word field is NULL only for the
 * leading and trailing margin words.
 */
static void wordgraph_create(Sentence const sent)
{
	Gword *new_word;

	assert(NULL==sent->last_word, "wordgraph_create(): wordgraph exists");
	new_word = gword_new(sent, sent->orig_sentence);

	assert(NULL!=sent->orig_sentence, "wordgraph_create()");
	assert(NULL==sent->wordgraph, "wordgraph_create(): wordgraph exists");

	sent->wordgraph = sent->last_word = new_word;
	new_word->label = "D"; /* dummy word */
	new_word->morpheme_type = MT_INFRASTRUCTURE;
}

/**
 * Create a trailing dummy "word" for the sentence.
 * It is a sentence "margin" trailing mark, in order to simplify the wordgraph
 * pointer manipulation (no need to check for NULL "next" pointer).
 * The unsplit_word field is NULL only for the margin words. */
static void wordgraph_terminator(Sentence const sent)
{
	assert(NULL != sent->last_word, "wordgraph_terminator(): no wordgraph");
	add_gword(sent, "(T)", NULL, MT_INFRASTRUCTURE); /* cannot use "" */
	sent->last_word->unsplit_word = NULL; /* no unsplit word */
	sent->last_word->label = "D"; /* dummy word */
	sent->last_word->tokenizing_step = TS_DONE; /* not to be tokenized */
}

/**
 * The string s has just been read in from standard input.
 * This function breaks it up into words and stores these words in
 * the sent->word[] array.  Returns true if all is well, false otherwise.
 */
bool separate_sentence(Sentence sent, Parse_Options opts)
{
	const char * word_end;
	//bool quote_found;
	Dictionary dict = sent->dict;
	mbstate_t mbs;
	const char * word_start = sent->orig_sentence;
	Gword *word;

	sent->length = 0;

	wordgraph_create(sent);

	if (dict->left_wall_defined)
		add_gword(sent, LEFT_WALL_WORD, NULL, MT_WALL);

	/* Reset the multibyte shift state to the initial state */
	memset(&mbs, 0, sizeof(mbs));

#ifdef DEBUG
	/* Skip a synthetic sentence mark, if any. See synthetic_split(). */
	if (SYNTHETIC_SENTENCE_MARK == sent->orig_sentence[0]) word_start++;
#endif

	for(;;)
	{
		wchar_t c;
		int nb = mbrtowc(&c, word_start, MB_CUR_MAX, &mbs);
		if (0 > nb) goto failure;

		while (is_space(c, dict->lctype))
		{
			word_start += nb;
			nb = mbrtowc(&c, word_start, MB_CUR_MAX, &mbs);
			if (0 == nb) break;
			if (0 > nb) goto failure;
		}

		if ('\0' == *word_start) break;

		/* Loop over non-blank characters until word-end is found. */
		word_end = word_start;
		nb = mbrtowc(&c, word_end, MB_CUR_MAX, &mbs);
		if (0 > nb) goto failure;
		while (!is_space(c, dict->lctype) && (c != 0) && (0 < nb))
		{
			word_end += nb;
			nb = mbrtowc(&c, word_end, MB_CUR_MAX, &mbs);
			if (0 > nb) goto failure;
		}

		/* FIXME: Morpheme type of initial bad-sentence word may be wrong.
		 * E.g: He 's here. (Space before ' so 's is classified as MT_WORD). */
		add_gword(sent, word_start, word_end, MT_WORD);
		word_start = word_end;
		if ('\0' == *word_start) break;
	}

	if (dict->right_wall_defined)
		add_gword(sent, RIGHT_WALL_WORD, NULL, MT_WALL);

	wordgraph_terminator(sent);

	while ((word = wordgraph_getqueue_word(sent)))
	{
		if (TS_DONE == word->tokenizing_step)
		{
			remqueue_gword(sent);
			continue;
		}

		/* Perform prefix, suffix splitting, if needed */
#ifdef DEBUG
		if (SYNTHETIC_SENTENCE_MARK == sent->orig_sentence[0])
			synthetic_split(sent, word);
#else
		if (0)
			;
#endif
		else
			separate_word(sent, word, opts);

		word->tokenizing_step = TS_DONE;
	}

	/* Return true if at least one sentence word has been issued */
	for (word = sent->wordgraph; NULL != word->next; word = word->next[0])
	{
		if ((word->morpheme_type != MT_INFRASTRUCTURE) &&
		    (word->morpheme_type != MT_WALL))
		{
			/* !test=wg or !test=wg:flags (for flags see wordgraph.h) */
			if (test_enabled("wg"))
				wordgraph_show(sent, test_enabled("wg"));
			return true;
		}
	}
	return false; /* Something is wrong */

failure:
#ifdef _WIN32
	prt_error("Unable to process UTF8 input string.\n");
#else
	prt_error("Unable to process UTF8 input string in current locale %s\n",
		nl_langinfo(CODESET));
#endif
	return false;
}

static Word *word_new(Sentence sent)
{
		const size_t len = sent->length;

		sent->word = realloc(sent->word, (len+1)*sizeof(*sent->word));
		sent->word[len].d= NULL;
		sent->word[len].x= NULL;
		sent->word[len].unsplit_word = NULL;
		sent->word[len].alternatives = NULL;
		sent->word[len].optional = false;
		sent->length++;

		return &sent->word[len];
}

/**
 * build_word_expressions() -- build list of expressions for a word.
 *
 * Looks up a word in the dictionary, fetching from it matching words and their
 * expressions.  Returns NULL if it's not there.  If there, it builds the list
 * of expressions for the word, and returns a pointer to it.
 * The subword of Gword w is used for this lookup, unless the subword is
 * explicitly given as parameter s. The subword of Gword w is always used as
 * the base word for each expression, and its subscript is the one from the
 * dictionary word of the expression.
 */
static X_node * build_word_expressions(Sentence sent, const Gword *w, const char *s)
{
	Dict_node * dn, *dn_head;
	X_node * x, * y;
	Exp_list eli;
	const Dictionary dict = sent->dict;

	eli.exp_list = NULL;
	dn_head = dictionary_lookup_list(dict, NULL == s ? w->subword : s);
	x = NULL;
	dn = dn_head;
	while (dn != NULL)
	{
		y = (X_node *) xalloc(sizeof(X_node));
		y->next = x;
		x = y;
		x->exp = copy_Exp(dn->exp);
		if (NULL == s)
		{
			x->string = dn->string;
		}
		else
		{
			dyn_str *xs = dyn_str_new();
			const char *sm = strrchr(dn->string, SUBSCRIPT_MARK);

			dyn_strcat(xs, w->subword);
			if (NULL != sm) dyn_strcat(xs, sm);
			x->string = string_set_add(xs->str, sent->string_set);
			dyn_str_delete(xs);
		}
		x->word = w;
		dn = dn->right;
	}
	free_lookup_list (dict, dn_head);
	free_Exp_list(&eli);
	return x;
}

/**
 * Build the expression lists for a given word at the current word-array word.
 *
 * The resulted word-array is later used as an input to the parser.
 *
 * Algorithm:
 * Apply the following step to all words w:
 *   - If w is in the dictionary, use it.
 *   - Else if w is identified by regex matching, use the appropriately
 *     matched disjunct collection.
 *   - Otherwise w is unknown - use the disjunct collection of UNKNOWN_WORD.
 *
 * FIXME For now, also add an element to the alternatives array, so the rest of
 * program will work fine (print_sentence_word_alternatives(),
 * sentence_in_dictionary(), verr_msg()).
 */
#define D_X_NODE 9
#define D_DWE 8
static bool determine_word_expressions(Sentence sent, Gword *w,
                                       unsigned int *ZZZ_added)
{
	Dictionary dict = sent->dict;
	const size_t wordpos = sent->length - 1;

	const char *s = w->subword;
	X_node * we = NULL;

	lgdebug(+D_DWE, "Word %zu subword %zu:'%s' status %s",
	        wordpos, w->node_num, s, gword_status(sent, w));
	if (NULL != sent->word[wordpos].unsplit_word)
		lgdebug(D_DWE, " (unsplit '%s')", sent->word[wordpos].unsplit_word);

	/* Generate an "alternatives" component. */
	altappend(sent, &sent->word[wordpos].alternatives, s);
	w->sent_wordidx = wordpos;

	if (w->status & WS_INDICT)
	{
		we = build_word_expressions(sent, w, NULL);
	}
	else if (w->status & WS_REGEX)
	{
		we = build_word_expressions(sent, w, w->regex_name);
	}
	else if (dict->unknown_word_defined && dict->use_unknown_word)
	{
		we = build_word_expressions(sent, w, UNKNOWN_WORD);
		assert(we, UNKNOWN_WORD " supposed to be defined in the dictionary!");
		w->status |= WS_UNKNOWN;
	}
	else
	{
		/* The word is unknown, but UNKNOWN_WORD cannot be used.
		 * An error message will eventually be printed. */
		prt_error("Error: Word '%s': word is unknown\n", w->subword);
		return false;
	}

#ifdef DEBUG
	assert(NULL != we, "Word '%s': NULL X-node", w->subword);
#else
	if (NULL == we)
	{
		/* FIXME Change it to assert() when the Wordgraph version is mature. */
		prt_error("Error: Word '%s': Internal error: NULL X_node", w->subword);
		return false;
	}
#endif

	/* If the current word is an empty-word (or like it), add a
	 * connector for an empty-word (EMPTY_CONNECTOR - ZZZ+) to the
	 * previous word. See the comments at add_empty_word().
	 * As a shortcut, only the first x-node is checked here for ZZZ-,
	 * supposing that the word has it in all of its dict entries
	 * (in any case, currently there is only 1 entry for each such word).
	 * Note that ZZZ_added starts by 0 and so also wordpos, and that the
	 * first sentence word (usually LEFT-WALL) doesn't need a check. */
	if ((wordpos != *ZZZ_added) && is_exp_like_empty_word(dict, we->exp))
	{
		lgdebug(D_DWE, " (has ZZZ-)");
		add_empty_word(dict, sent->word[wordpos-1].x);
		*ZZZ_added = wordpos; /* Remember it for not doing it again */
	}
	lgdebug(D_DWE, "\n");

	/* At last .. concatenate the word expressions we build for
	 * this alternative. */
	sent->word[wordpos].x = catenate_X_nodes(sent->word[wordpos].x, we);
	if (verbosity_level(D_X_NODE))
	{
		/* Print the X_node details for the word. */
		prt_error("Debug: Tokenize word/alt=%zu/%zu '%s' re=%s\n\\",
				 wordpos, altlen(sent->word[wordpos].alternatives), s,
				 w->regex_name ? w->regex_name : "");
		while (we)
		{
			prt_error("Debug:  string='%s' expr=", we->string);
			print_expression(we->exp);
			we = we->next;
		}
	}

	return true;
}
#undef D_DWE

#if 0 /* unused */
/**
 * Find whether w1 and w2 have been generated together in the same alternative.
 */
static bool is_alternative_next_word(const Gword *w1, const Gword *w2)
{
	assert(NULL != w1->alternative_id, "Word '%s' NULL alternative_id",
	       w1->subword);
	lgdebug(+6, "w1='%s' (%p=%s) w2='%s' (%p=%s) \n",
	        w1->subword, w1->alternative_id, w1->alternative_id->subword,
	        w2->subword, w2->alternative_id, w2->alternative_id->subword);
	return (w1->alternative_id == w2->alternative_id);
}
#endif

#ifdef FIXIT /* unused */
/* XXX WS_UNSPLIT */
static bool same_unsplit_word(Sentence sent, const Gword *w1, const Gword *w2)
{
	return ((w1->unsplit_word == w2->unsplit_word) &&
	        (w1->unsplit_word != sent->wordgraph));
}
#endif

#define D_WPP 8
static void print_wordgraph_pathpos(const Wordgraph_pathpos *wp)
{
	size_t i = 0;

	if (NULL == wp)
	{
		lgdebug(+D_WPP, "Empty\n");
		return;
	}
	lgdebug(+D_WPP, "\n");
	for (; NULL != wp->word; wp++)
	{
		lgdebug(D_WPP, "%zu: %zu:word '%s', same=%d used=%d level=%zu\n",
		        i++, wp->word->node_num, wp->word->subword, wp->same_word,
		        wp->used, wp->word->hier_depth);
	}
}
#undef D_WPP

/**
 * "Flatten" the wordgraph into a word array.
 * Return false if an error was encountered.
 */
#define D_FW 8
bool flatten_wordgraph(Sentence sent, Parse_Options opts)
{
	Wordgraph_pathpos *wp_new = NULL;
	Wordgraph_pathpos *wp_old = NULL;
	Wordgraph_pathpos *wpp_new, *wpp_old;
	Gword *wg_word;               /* A wordgraph word */
	Gword **next;                 /* The next words */
	const Gword *last_unsplit_word = NULL;
	size_t max_words = 0;
	bool error_encountered = false;
	bool right_wall_encountered = false;
	unsigned int ZZZ_added = 0;   /* ZZZ+ has been added to previous word */

	assert(0 == sent->length, "flatten_wordgraph(): Word array already exists.");

	/* Establish an upper bound on the total number of words, to prevent an
	 * infinite loop in case of a bug. At the same time, calculate the
	 * hierarchy position of the word. */
	for (wg_word = sent->wordgraph->chain_next; wg_word;
	     wg_word = wg_word->chain_next)
	{
		wordgraph_hier_position(wg_word);
		max_words++;
	}

	/* Populate the pathpos word queue */
	for (next = sent->wordgraph->next; *next; next++)
	{
		wordgraph_pathpos_add(&wp_new, *next,
		                      false/* used */, false/* same_word */,
		                      true/* diff_alternative */);
	}

	/* Scan the wordgraph and flatten it. */
	do
	{
		Word *wa_word; /* A word-array word (for the parsing stage) */
		const Gword *unsplit_word;

		assert(NULL != wp_new, "pathpos word queue is empty");
		wp_old = wp_new;
		wp_new = NULL;
		print_wordgraph_pathpos(wp_old);

		/* Add a new word to the sentence word array.
		 */
		assert(0 < max_words--, "Too many words (it may be an infinite loop)");
		wa_word = word_new(sent);

		/* Go upward and find the sentence word. */
		unsplit_word  = wp_old->word;
		if (MT_INFRASTRUCTURE != unsplit_word->morpheme_type)
		{
			while (!IS_SENTENCE_WORD(sent, unsplit_word))
			{
				assert(NULL != unsplit_word, "'%s': Unsplit word not found",
						 wg_word->subword);
				unsplit_word = unsplit_word->unsplit_word;
			}

			assert(NULL != unsplit_word->subword, "Unsplit word not found");

			if (unsplit_word != last_unsplit_word)
			{
				/* This is a new sentence word - use it as the unsplit word. */
				wa_word->unsplit_word = unsplit_word->subword;
				last_unsplit_word = unsplit_word;
			}
		}

		/* Generate the X-nodes. */
		for (wpp_old = wp_old; NULL != wpp_old->word; wpp_old++)
		{
			wg_word = wpp_old->word;
			if (NULL == wg_word->next) continue; /* XXX avoid termination */

			if (wpp_old->same_word)
			{
				/* We haven't advanced to the next wordgraph word, so its X-node
				 * has already been generated in a previous word of the word
				 * array.  This means we are in a longer alternative which has
				 * "extra" words that may not have links, and this is one of
				 * them.  Mark it as "optional", so we consider that while
				 * parsing, and then remove it in case it doesn't have links. */
				sent->word[sent->length - 1].optional = true;
			}
			else
			{
				/* Words are not supposed to get issued more than once. */
				assert(!wpp_old->used, "Word %zu:%s has been used",
				       wg_word->node_num, wpp_old->word->subword);

				/* This is a new wordgraph word.
				 */
				assert(!right_wall_encountered, "Extra word");
				if (!determine_word_expressions(sent, wg_word, &ZZZ_added))
					error_encountered = true;
				if ((MT_WALL == wg_word->morpheme_type) &&
				    0== strcmp(wg_word->subword, RIGHT_WALL_WORD))
					right_wall_encountered = true;
				wpp_old->used = true;
			}
		}

		/* Scan the old pathpos queue, and check for which words we can advance
		 * in the wordgraph. Do it in two passes:
		 * 1. Advance to next words that are next in the alternative of old
		 * words.
		 * 2. Advance to next words that are in a different alternative than the
		 * words that are already in the new pathpos queue.
		 */

		for (wpp_old = wp_old; NULL != wpp_old->word; wpp_old++)
		{
			wg_word = wpp_old->word;
			if (NULL == wg_word->next) continue; /* XXX avoid termination word */

			/* Here wg_word->next cannot be NULL. */
			assert(NULL != wg_word->next[0], "Bad wordgraph: "
			       "'%s'->next[0]==NULL", wg_word->subword);
			assert((NULL != wg_word->next[0]->prev)
					 || (NULL != wg_word->next[0]->next),  "Bad wordgraph: "
			       "'%s'->next[0]->prev/next==NULL", wg_word->subword);
			assert(NULL != wg_word->next[0]->prev[0], "Bad wordgraph: "
			       "'%s'->next[0]->prev[0]==NULL", wg_word->subword);

			for (next = wg_word->next; NULL != *next; next++)
			{
				if (wg_word->hier_depth <= (*next)->hier_depth &&
				    (NULL == (*next)->prev[1]))
				{
					lgdebug(+D_FW, "Word %zu:%s(%zu) next %zu:%s(%zu) next_ok\n",
					        wg_word->node_num, wg_word->subword, wg_word->hier_depth,
					        (*next)->node_num, (*next)->subword, (*next)->hier_depth);
					wpp_old->next_ok = true;
					break;
				}
			}

			if (wpp_old->next_ok)
			{
				lgdebug(+D_FW, "Advancing %zu:%s next_ok\n", wg_word->node_num,
				        wg_word->subword);
				for (next = wg_word->next; NULL != *next; next++)
				{
					wordgraph_pathpos_add(&wp_new, *next,
					                      false/* used */, false/* same_word */,
					                      true/* diff_alternative */);
				}
			}
		}

		for (wpp_old = wp_old; NULL != wpp_old->word; wpp_old++)
		{
			wg_word = wpp_old->word;

			if (!wpp_old->next_ok) /* next_ok words have got handled above */
			{
				bool same_alternative = false;

				if (NULL == wg_word->next) continue; /* termination word */

				if (NULL != wp_new)
				{
					for (next = wg_word->next; NULL != *next; next++)
					{
						for (wpp_new = wp_new; NULL != wpp_new->word; wpp_new++)
						{
							if ((wpp_new->word != *next) &&
							    in_same_alternative(wpp_new->word, *next))
							{
								lgdebug(+D_FW, "same_alternative: %zu:%s and %zu:%s\n",
								        wpp_new->word->node_num, wpp_new->word->subword,
								        (*next)->node_num, (*next)->subword);
								same_alternative = true;
								break;
							}
						}
						if (same_alternative) break; /* shortcut */
					}
				}

				/* If there are already words in the pathpos queue from the same
				 * alternative of the common ancestor of the next word, we cannot
				 * put it yet in the queue, because we should not put in the same
				 * slot of the word-array, words from the same alternative since
				 * else only one of them can be chosen by the linkage. Hence put
				 * again in the pathpos queue the current word, marking it was
				 * "same_word". This will cause generation of an empty word in the
				 * next round. */
				lgdebug(+D_FW, "Advancing %zu:%s: ", wg_word->node_num,
				        wg_word->subword);

				if (same_alternative)
				{
					lgdebug(D_FW, "No (same alt) used=%d\n", wpp_old->used);
					wordgraph_pathpos_add(&wp_new, wg_word,
					                      wpp_old->used, true/* same_word */,
					                      true/* diff_alternative */);
				}
				else
				{
					bool added = false;

					for (next = wg_word->next; NULL != *next; next++)
						added |= wordgraph_pathpos_add(&wp_new, *next,
						                               false/* used */,
						                               false/* same_word */,
						                               true/* diff_alternative */);
					if (added)
					{
						lgdebug(D_FW, "Yes\n");
					}
					else
					{
						lgdebug(D_FW, "No (existing)\n");
					}
				}
			}
		}

		free(wp_old);
	} while ((NULL != wp_new[1].word) ||
	         (wp_new[0].word->morpheme_type != MT_INFRASTRUCTURE));

	wp_new[0].word->sent_wordidx = sent->length;
	free(wp_new);
	lgdebug(+D_FW, "sent->length %zu\n", sent->length);
	if (verbosity_level(D_SW))
	{
		dyn_str *s = dyn_str_new();
		print_sentence_word_alternatives(s, sent, true, NULL, NULL);
		char *out = dyn_str_take(s);
		prt_error("Debug: Sentence words and alternatives:\n%s", out);
		free(out);
	}

	return !error_encountered;
}
#undef D_FW

/**
 * This just looks up all the words in the sentence, and builds
 * up an appropriate error message in case some are not there.
 * It has no side effect on the sentence.  Returns true if all
 * went well.
 *
 * This code is called only if the 'unknown-words' flag is set.
 */
bool sentence_in_dictionary(Sentence sent)
{
	bool ok_so_far;
	size_t w;
	const char * s;
	Dictionary dict = sent->dict;
	char temp[1024];

	ok_so_far = true;
	for (w=0; w<sent->length; w++)
	{
		size_t ialt;
		for (ialt=0; NULL != sent->word[w].alternatives[ialt]; ialt++)
		{
			s = sent->word[w].alternatives[ialt];
			if (!find_word_in_dict(dict, s))
			{
				if (ok_so_far)
				{
					safe_strcpy(temp, "The following words are not in the dictionary:", sizeof(temp));
					ok_so_far = false;
				}
				safe_strcat(temp, " \"", sizeof(temp));
				safe_strcat(temp, s, sizeof(temp));
				safe_strcat(temp, "\"", sizeof(temp));
			}
		}
	}
	if (!ok_so_far)
	{
		err_ctxt ec = { sent };
		err_msgc(&ec, lg_Error, "Sentence not in dictionary\n%s\n", temp);
	}
	return ok_so_far;
}
