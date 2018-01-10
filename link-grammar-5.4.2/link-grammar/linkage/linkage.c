/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2008, 2009, 2013, 2014 Linas Vepstas                        */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <stdint.h>
#include <stdlib.h>

#include "api-structures.h"
#include "connectors.h"
#include "dict-common/dict-affix.h"  // for INFIX_MARK from dict.
#include "dict-common/dict-defines.h" // for SUBSCRIPT_MARK
#include "dict-common/idiom.h"
#include "disjunct-utils.h"
#include "link-includes.h"
#include "linkage.h"
#include "lisjuncts.h"
#include "post-process/post-process.h" // needed only for exfree_domain_names
#include "post-process/pp-structures.h" // needed only for pp_info_s
#include "sat-solver/sat-encoder.h"
#include "string-set.h"
#include "tokenize/wordgraph.h"
#include "tokenize/tok-structures.h" // XXX TODO provide gword access methods!
#include "tokenize/word-structures.h" // For Word_struct

#define INFIX_MARK_L 1 /* INFIX_MARK is 1 character */
#define STEM_MARK_L  1 /* stem mark is 1 character */

/* Marks around a null word. */
#define NULLWORD_START '['
#define NULLWORD_END   ']'

/**
 * Append an unmarked (i.e. without INFIXMARK) morpheme to join_buff.
 * join_buff is a zeroed-out buffer which has enough room for morpheme to be
 * added + terminating NUL.
 * Note that MT_PREFIX or MT_SUFFIX can be without an INFIX_MARK, in case
 * INFIX_MARK is not defined. XXX: What about MT_MIDDLE? (not in use yet).
 *
 * FIXME Combining contracted words is not handled yet, because combining
 * morphemes which have non-LL links to other words is not yet implemented.
 */
static void add_morpheme_unmarked(Sentence sent, char *join_buff,
                                  const char *wm, Morpheme_type mt)
{
	const char infix_mark = INFIX_MARK(sent->dict->affix_table);
	const char *sm =  strrchr(wm, SUBSCRIPT_MARK);

	if (NULL == sm) sm = (char *)wm + strlen(wm);

	if ((MT_PREFIX == mt) && (infix_mark == sm[-INFIX_MARK_L]))
		strncat(join_buff, wm, sm-wm-INFIX_MARK_L);
	else if ((MT_SUFFIX == mt) && (infix_mark == wm[0]))
		strncat(join_buff, INFIX_MARK_L+wm, sm-wm-INFIX_MARK_L);
	else if ((MT_MIDDLE == mt))
		strncat(join_buff, INFIX_MARK_L+wm, sm-wm-2*INFIX_MARK_L);
	else
		strncat(join_buff, wm, sm-wm);
}

static const char *join_null_word(Sentence sent, Gword **wgp, size_t count)
{
	size_t i;
	char *join_buff;
	const char *s;
	size_t join_len = 0;

	for (i = 0; i < count; i++)
		join_len += strlen(wgp[i]->subword);

	join_buff = alloca(join_len+1);
	memset(join_buff, '\0', join_len+1);

	for (i = 0; i < count; i++)
		add_morpheme_unmarked(sent, join_buff, wgp[i]->subword,
		                      wgp[i]->morpheme_type);

	s = string_set_add(join_buff, sent->string_set);

	return s;
}

/**
 * Add a null word node that represents two or more null morphemes.
 * Used for "unifying" null morphemes that are part of a single subword,
 * when only some of its morphemes (2 or more) don't have a linkage.
 * The words "start" to "end" (including) are unified by the new node.
 */
static Gword *wordgraph_null_join(Sentence sent, Gword **start, Gword **end)
{
	Gword *new_word;
	Gword **w;
	char *usubword;
	size_t join_len = 0;

	for (w = start; w <= end; w++) join_len += strlen((*w)->subword);
	usubword = calloc(join_len+1, 1); /* zeroed out */

	for (w = start; w <= end; w++)
		add_morpheme_unmarked(sent, usubword, (*w)->subword, (*w)->morpheme_type);

	new_word = gword_new(sent, usubword);
	free(usubword);
	new_word->status |= WS_PL;
	new_word->label = "NJ";
	new_word->null_subwords = NULL;
	new_word->start = (*start)->start;
	new_word->end = (*end)->end;

	/* Link the null_subwords links of the added unifying node to the null
	 * subwords it unified. */
	for (w = start; w <= end; w++)
		gwordlist_append(&new_word->null_subwords, (Gword *)(*w));
	/* Removing const qualifier, but gwordlist_append doesn't change w->... .  */

	return new_word;
}

/**
 * The functions defined in this file are primarily a part of the user API
 * for working with linkages.
 */

#define SUBSCRIPT_SEP SUBSCRIPT_DOT /* multiple-subscript separator */

#define PREFIX_SUPPRESS ("PL") /* prefix links start with this */
#define PREFIX_SUPPRESS_L 2    /* length of above */
#define SUFFIX_SUPPRESS ("LL") /* suffix links start with this */
#define SUFFIX_SUPPRESS_L 2    /* length of above */

#define HIDE_MORPHO   (!display_morphology)
/* TODO? !display_guess_marks is not implemented. */
#define DISPLAY_GUESS_MARKS true // (opts->display_guess_marks)

/* FIXME: Define an affix class MORPHOLOGY_LINKS. */
static inline bool is_morphology_link(const char *link_name)
{
	if (NULL == link_name) return false;
	return (0 == strncmp(link_name, SUFFIX_SUPPRESS, SUFFIX_SUPPRESS_L)) ||
	       (0 == strncmp(link_name, PREFIX_SUPPRESS, PREFIX_SUPPRESS_L));
}

/*
 * Remap the link array according to discarded links and words.
 *
 * The remap[] elements indicate the new WordIdx of the word.
 * A value which is -1 indicates a discarded word.
 * A NULL link_name indicates a discarded link.
 */
static void remap_linkages(Linkage lkg, const int *remap)
{
	LinkIdx i, j;

	for (i = 0, j = 0; i < lkg->num_links; i++)
	{
		Link *old_lnk = &lkg->link_array[i];

		if (NULL != old_lnk->link_name &&  /* discarded link */
		   (-1 != remap[old_lnk->rw]) && (-1 != remap[old_lnk->lw]))
		{
			Link *new_lnk = &lkg->link_array[j];
			Connector *ctmp;

			/* Copy the entire link contents, thunking the word numbers.
			 * Note that j is always <= i so this is always safe. */

			new_lnk->lw = remap[old_lnk->lw];
			new_lnk->rw = remap[old_lnk->rw];

			ctmp = new_lnk->lc;
			new_lnk->lc = old_lnk->lc;
			old_lnk->lc = ctmp;

			ctmp = new_lnk->rc;
			new_lnk->rc = old_lnk->rc;
			old_lnk->rc = ctmp;

			new_lnk->link_name = old_lnk->link_name;

			/* Remap the pp_info, too. */
			if (lkg->pp_info)
				lkg->pp_info[j] = lkg->pp_info[i];

			j++;
		}
		else
		{
			/* Whack this slot of pp_info. */
			if (lkg->pp_info)
				exfree_domain_names(&lkg->pp_info[i]);
		}
	}

	lkg->num_links = j;
	/* Unused memory not freed - all of it will be freed in free_linkages(). */
}

/**
 * Remove unlinked optional words from a linkage.
 * XXX Should we remove here also the dict-cap tokens? In any case, for now they
 * are left for debug.
 */
#define D_REE 7
void remove_empty_words(Linkage lkg)
{
	size_t i, j;
	Disjunct **cdj = lkg->chosen_disjuncts;
	int *remap = alloca(lkg->num_words * sizeof(*remap));
	Gword **wgp = lkg->wg_path;

	for (i = 0, j = 0; i < lkg->num_words; i++)
	{
		/* Discard optional words that are not real null-words.  Note that
		 * if optional words don't have non-optional words after them,
		 * wg_path doesn't include them, and hence *wgp is NULL then. */
		if ((NULL == *wgp) || ((*wgp)->sent_wordidx != i))
		{
			assert((NULL == cdj[i]) && lkg->sent->word[i].optional);
			remap[i] = -1;
			continue;
		}

		Disjunct *cdtmp = cdj[j];
		cdj[j] = cdj[i];
		cdj[i] = cdtmp; /* The SAT parser frees chosen_disjuncts elements. */
		remap[i] = j;
		j++;
		wgp++;
	}
	if (lkg->num_words != j)
	{
		/* Unused memory not freed - all of it will be freed in free_linkages(). */
		lkg->num_words = j;
		remap_linkages(lkg, remap); /* Update lkg->link_array and lkg->num_links. */
	}
}
#undef D_REE

/**
 * This takes the Wordgraph path array and uses it to
 * compute the chosen_words array.  "I.xx" suffixes are eliminated.
 *
 * chosen_words
 *    A pointer to an array of pointers to strings.  These are the words to be
 *    displayed when printing the solution, the links, etc.  Computed as a
 *    function of chosen_disjuncts[] by compute_chosen_words().  This differs
 *    from sentence.word[].alternatives because it contains the subscripts.  It
 *    differs from chosen_disjunct[].string in that the idiom symbols have been
 *    removed.  Furthermore, several chosen_disjuncts[].string elements may be
 *    combined into one chosen_words[] element if opts->display_morphology==0 or
 *    that they where linkage null-words that are morphemes of the same original
 *    word (i.e. subwords of an unsplit_word which are marked as morphemes).
 *
 * wg_path
 *    A pointer to a NULL-terminated array of pointers to Wordgraph words.
 *    It corresponds 1-1 to the chosen_disjuncts array in Linkage structure.
 *    A new one is constructed below to correspond 1-1 to chosen_words.
 *
 *    FIXME Sometimes the word strings are taken from chosen_disjuncts,
 *    and sometimes from wordgraph subwords.
 */
#define D_CCW 8
void compute_chosen_words(Sentence sent, Linkage linkage, Parse_Options opts)
{
	WordIdx i;   /* index of chosen_words */
	WordIdx j;
	Disjunct **cdjp = linkage->chosen_disjuncts;
	const char **chosen_words = alloca(linkage->num_words * sizeof(*chosen_words));
	int *remap = alloca(linkage->num_words * sizeof(*remap));
	bool *show_word = alloca(linkage->num_words * sizeof(*show_word));
	bool display_morphology = opts->display_morphology;

	Gword **lwg_path = linkage->wg_path;
	Gword **n_lwg_path = NULL; /* new Wordgraph path, to match chosen_words */

	Gword **nullblock_start = NULL; /* start of a null block, to be put in [] */
	size_t nbsize = 0;              /* number of word in a null block */
	Gword *unsplit_word = NULL;

	memset(show_word, 0, linkage->num_words * sizeof(*show_word));

	if (verbosity_level(D_CCW))
		print_lwg_path(lwg_path, "Linkage");

	for (i = 0; i < linkage->num_words; i++)
	{
		Disjunct *cdj = cdjp[i];
		Gword *w;              /* current word */
		const Gword *nw;       /* next word (NULL if none) */
		Gword **wgp;           /* wordgraph_path traversing pointer */

		const char *t = NULL;  /* current word string */
		bool nb_end;           /* current word is at end of a nullblock */
		bool join_alt = false; /* morpheme-join this alternative */
		char *s;
		size_t l;
		size_t m;

		lgdebug(D_CCW, "Loop start, word%zu: cdj %s, path %s\n",
		        i, cdj ? cdj->word_string : "NULL",
		        lwg_path[i] ? lwg_path[i]->subword : "NULL");

		w = lwg_path[i];
		nw = lwg_path[i+1];
		wgp = &lwg_path[i];
		unsplit_word = w->unsplit_word;

		/* FIXME If the original word was capitalized in a capitalizable
		 * position, the displayed null word may be its downcase version. */

		if (NULL == cdj) /* a null word (the chosen disjunct was NULL) */
		{
			nbsize++;
			if (NULL == nullblock_start) /* it starts a new null block */
				nullblock_start = wgp;

			nb_end = (NULL == nw) || (nw->unsplit_word != unsplit_word) ||
				(MT_INFRASTRUCTURE == w->unsplit_word->morpheme_type);

			/* Accumulate null words in this alternative */
			if (!nb_end && (NULL == cdjp[i+1]))
			{
				lgdebug(D_CCW, "Skipping word%zu cdjp=NULL#%zu, path %s\n",
				         i, nbsize, lwg_path[i]->subword);
				chosen_words[i] = NULL;
				continue;
			}

			if (NULL != nullblock_start)
			{
				/* If we are here, this null word is an end of a null block */
				lgdebug(+D_CCW, "Handling %zu null words at %zu: ", nbsize, i);

				if (1 == nbsize)
				{
					/* Case 1: A single null subword. */
					lgdebug(D_CCW, "A single null subword.\n");
					t = join_null_word(sent, wgp, nbsize);

					gwordlist_append(&n_lwg_path, w);
				}
				else
				{
					lgdebug(D_CCW, "Combining null subwords");
					/* Use alternative_id to check for start of alternative. */
					if (((*nullblock_start)->alternative_id == *nullblock_start)
					    && nb_end)
					{
						/* Case 2: A null unsplit_word (all-nulls alternative).*/
						lgdebug(D_CCW, " (null alternative)\n");
						t = unsplit_word->subword;

						gwordlist_append(&n_lwg_path, unsplit_word);
					}
					else
					{
						/* Case 3: Join together >=2 null morphemes. */
						Gword *wgnull;

						lgdebug(D_CCW, " (null partial word)\n");
						wgnull = wordgraph_null_join(sent, wgp-nbsize+1, wgp);
						gwordlist_append(&n_lwg_path, wgnull);
						t = wgnull->subword;
					}
				}

				nullblock_start = NULL;
				nbsize = 0;
				show_word[i] = true;

				if (MT_WALL != w->morpheme_type)
				{
					/* Put brackets around the null word. */
					l = strlen(t) + 2;
					s = (char *) alloca(l+1);
					s[0] = NULLWORD_START;
					strcpy(&s[1], t);
					s[l-1] = NULLWORD_END;
					s[l] = '\0';
					t = string_set_add(s, sent->string_set);
					lgdebug(D_CCW, " %s\n", t);
					/* Null words have no links, so take care not to drop them. */
				}
			}
		}
		else
		{
			/* This word has a linkage. */

			/* TODO: Suppress "virtual-morphemes", currently the dictcap ones. */
			char *sm;

			t = cdj->word_string;
			/* Print the subscript, as in "dog.n" as opposed to "dog". */

			if (0)
			{
				/* TODO */
			}
			else
			{
				/* Get rid of those ugly ".Ixx" */
				if (is_idiom_word(t))
				{
					s = strdupa(t);
					sm = strrchr(s, SUBSCRIPT_MARK);
					*sm = '\0';
					t = string_set_add(s, sent->string_set);
				}
				else if (HIDE_MORPHO)
				{
					/* Concatenate the word morphemes together into one word.
					 * Concatenate their subscripts into one subscript.
					 * Use subscript separator SUBSCRIPT_SEP.
					 * XXX Check whether we can encounter an idiom word here.
					 * FIXME Combining contracted words is not handled yet, because
					 * combining morphemes which have non-LL links to other words is
					 * not yet implemented.
					 * FIXME Move to a separate function. */
					Gword **wgaltp;
					size_t join_len = 0;
					size_t mcnt = 0;

					/* If the alternative contains morpheme subwords, mark it
					 * for joining... */
					for (wgaltp = wgp, j = i; NULL != *wgaltp; wgaltp++, j++)
					{

						if ((*wgaltp)->unsplit_word != unsplit_word) break;
						if (MT_INFRASTRUCTURE ==
						    (*wgaltp)->unsplit_word->morpheme_type) break;

						mcnt++;

						if (NULL == cdjp[j])
						{
							/* ... but not if it contains a null word */
							join_alt = false;
							break;
						}
						join_len += strlen(cdjp[j]->word_string) + 1;
						if ((*wgaltp)->morpheme_type & IS_REG_MORPHEME)
							join_alt = true;
					}

					if (join_alt)
					{
						/* Join it in two steps: 1. Base words. 2. Subscripts.
						 * FIXME? Can be done in one step (more efficient but maybe
						 * less clear).
						 * Put SUBSCRIPT_SEP between the subscripts.
						 * XXX No 1-1 correspondence between the hidden base words
						 * and the subscripts after the join, in case there are base
						 * words with and without subscripts. */

						const char subscript_sep_str[] = { SUBSCRIPT_SEP, '\0'};
						const char subscript_mark_str[] = { SUBSCRIPT_MARK, '\0'};
						char *join = calloc(join_len + 1, 1); /* zeroed out */

						join[0] = '\0';

						/* 1. Join base words. (Could just use the unsplit_word.) */
						for (wgaltp = wgp, m = 0; m < mcnt; wgaltp++, m++)
						{
							add_morpheme_unmarked(sent, join, cdjp[i+m]->word_string,
							                      (*wgaltp)->morpheme_type);
						}

						strcat(join, subscript_mark_str); /* tentative */

						/* 2. Join subscripts. */
						for (wgaltp = wgp, m = 0; m < mcnt; wgaltp++, m++)
						{
							/* Cannot NULLify the word - we may have links to it. */
							if (m != mcnt-1) chosen_words[i+m] = "";

							sm =  strrchr(cdjp[i+m]->word_string, SUBSCRIPT_MARK);

							if (NULL != sm)
							{
								/* Supposing stem subscript is .=x (x optional) */
								if (MT_STEM == (*wgaltp)->morpheme_type)
								{
									sm += 1 + STEM_MARK_L; /* sm+strlen(".=") */
									if ('\0' == *sm) sm = NULL;
#if 0
									if ((cnt-1) == m)
									{
										/* Support a prefix-stem combination. In that case
										 * we have just nullified the combined word, so we
										 * need to move it to the position of the prefix.
										 * FIXME: May still not be good enough. */
										move_combined_word = i+m-1;

										/* And the later chosen_word assignment should be:
										 * chosen_words[-1 != move_combined_word ?
										 *    move_combined_word : i] = t;
										 */
									}
									else
									{
										move_combined_word = -1;
									}
#endif
								}
							}
							if (NULL != sm)
							{
								strcat(join, sm+1);
								strcat(join, subscript_sep_str);
							}
						}

						/* Remove an extra mark, if any */
						join_len = strlen(join);
						if ((SUBSCRIPT_SEP == join[join_len-1]) ||
							 (SUBSCRIPT_MARK == join[join_len-1]))
							join[join_len-1] = '\0';

						gwordlist_append(&n_lwg_path, unsplit_word);
						t = string_set_add(join, sent->string_set);
						free(join);

						i += mcnt-1;
					}
				}
			}

			if (!join_alt) gwordlist_append(&n_lwg_path, *wgp);

			/*
			 * Add guess marks in [] square brackets, if needed, at the
			 * end of the base word. Convert the badly-printing
			 * SUBSCRIPT_MARK (hex 03 or ^C) into a period.
			 */
			if (t)
			{

				s = strdupa(t);
				sm = strrchr(s, SUBSCRIPT_MARK);
				if (sm) *sm = SUBSCRIPT_DOT;

				if ((!(w->status & WS_GUESS) && (w->status & WS_INDICT))
				    || !DISPLAY_GUESS_MARKS)
				{
					t = string_set_add(s, sent->string_set);
				}
				else
				{
					const char *regex_name = w->regex_name;
					/* 4 = 1(null) + 1(guess_mark) + 2 (sizeof "[]") */
					int baselen = NULL == sm ? strlen(t) : (size_t)(sm-s);
					char guess_mark = 0;

					switch (w->status & WS_GUESS)
					{
						case WS_SPELL:
							guess_mark = GM_SPELL;
							break;
						case WS_RUNON:
							guess_mark = GM_RUNON;
							break;
						case WS_REGEX:
							guess_mark = GM_REGEX;
							break;
						case 0:
							guess_mark = GM_UNKNOWN;
							break;
						default:
							assert(0, "Missing 'case: %2x'", w->status & WS_GUESS);
					}

					/* In the case of display_morphology==0, the guess indication of
					 * the last subword is used as the guess indication of the whole
					 * word.
					 * FIXME? The guess indications of other subwords are ignored in
					 * this mode. This implies that if a first or middle subword has
					 * a guess indication but the last subword doesn't have, no guess
					 * indication would be shown at all. */

					if ((NULL == regex_name) || HIDE_MORPHO) regex_name = "";
					s = alloca(strlen(t) + strlen(regex_name) + 4);
					strncpy(s, t, baselen);
					s[baselen] = '[';
					s[baselen + 1] = guess_mark;
					strcpy(s + baselen + 2, regex_name);
					strcat(s, "]");
					if (NULL != sm) strcat(s, sm);
					t = string_set_add(s, sent->string_set);
				}
			}
		}

		assert(t != NULL, "Word %zu: NULL", i);
		chosen_words[i] = t;
	}

	/* Conditional test removal of quotation marks and the "capdict" tokens,
	 * to facilitate using diff on sentence batch runs. */
	if (test_enabled("removeZZZ"))
	{
		for (i=0, j=0; i<linkage->num_links; i++)
		{
			Link *lnk = &(linkage->link_array[i]);

			if (0 == strcmp("ZZZ", lnk->link_name))
				chosen_words[lnk->rw] = NULL;
		}
	}

	/* If morphology printing is being suppressed, then all links
	 * connecting morphemes will be discarded. */
	if (HIDE_MORPHO)
	{
		/* Discard morphology links. */
		for (i=0; i<linkage->num_links; i++)
		{
			Link * lnk = &linkage->link_array[i];

			if (is_morphology_link(lnk->link_name))
			{
				/* Mark link for discarding. */
				lnk->link_name = NULL;
			}
			else
			{
				/* Mark word for not discarding. */
				show_word[lnk->rw] = true;
				show_word[lnk->lw] = true;
			}
		}
	}

	/* We alloc a little more than needed, but so what... */
	linkage->word = (const char **) exalloc(linkage->num_words*sizeof(char *));

	/* Copy over the chosen words, dropping the discarded words.
	 * However, don't discard existing words (chosen_words[i][0]).
	 * Note that if a word only has morphology links and is not combined with
	 * another word, then it will get displayed with no links at all (e.g.
	 * when explicitly specifying root and suffix for debug: root.= =suf */
	for (i=0, j=0; i<linkage->num_words; ++i)
	{
		if (chosen_words[i] &&
		    (chosen_words[i][0] || (!HIDE_MORPHO || show_word[i])))
		{
			const char *cwtmp = linkage->word[j];
			linkage->word[j] = chosen_words[i];
			chosen_words[i] = cwtmp;
			remap[i] = j;
			j++;
		}
		else
		{
			remap[i] = -1;
		}
	}
	linkage->num_words = j;

	remap_linkages(linkage, remap); /* Update linkage->link_array / num_links. */

	linkage->wg_path_display = n_lwg_path;

	if (verbosity_level(D_CCW))
		print_lwg_path(n_lwg_path, "Display");
}
#undef D_CCW

Linkage linkage_create(LinkageIdx k, Sentence sent, Parse_Options opts)
{
	Linkage linkage;

	if (opts->use_sat_solver)
	{
		linkage = sat_create_linkage(k, sent, opts);
		if (!linkage) return NULL;
	}
	else
	{
		/* Cannot create a Linkage for a discarded linkage. */
		if (sent->num_linkages_post_processed <= k) return NULL;
		linkage = &sent->lnkages[k];
	}

	/* Perform remaining initialization we haven't done yet...*/
	compute_chosen_words(sent, linkage, opts);

	linkage->is_sent_long = (linkage->num_words >= opts->twopass_length);

	return linkage;
}

void linkage_delete(Linkage linkage)
{
	/* Currently a no-op */
}

size_t linkage_get_num_words(const Linkage linkage)
{
	if (!linkage) return 0;
	return linkage->num_words;
}

size_t linkage_get_num_links(const Linkage linkage)
{
	if (!linkage) return 0;
	return linkage->num_links;
}

static inline bool verify_link_index(const Linkage linkage, LinkIdx index)
{
	if (!linkage) return false;
	if (index >= linkage->num_links) return false;
	return true;
}

int linkage_get_link_length(const Linkage linkage, LinkIdx index)
{
	Link *link;
	if (!verify_link_index(linkage, index)) return -1;
	link = &(linkage->link_array[index]);
	return link->rw - link->lw;
}

WordIdx linkage_get_link_lword(const Linkage linkage, LinkIdx index)
{
	if (!verify_link_index(linkage, index)) return SIZE_MAX;
	return linkage->link_array[index].lw;
}

WordIdx linkage_get_link_rword(const Linkage linkage, LinkIdx index)
{
	if (!verify_link_index(linkage, index)) return SIZE_MAX;
	return linkage->link_array[index].rw;
}

const char * linkage_get_link_label(const Linkage linkage, LinkIdx index)
{
	if (!verify_link_index(linkage, index)) return NULL;
	return linkage->link_array[index].link_name;
}

const char * linkage_get_link_llabel(const Linkage linkage, LinkIdx index)
{
	if (!verify_link_index(linkage, index)) return NULL;
	return linkage->link_array[index].lc->string;
}

const char * linkage_get_link_rlabel(const Linkage linkage, LinkIdx index)
{
	if (!verify_link_index(linkage, index)) return NULL;
	return linkage->link_array[index].rc->string;
}

const char ** linkage_get_words(const Linkage linkage)
{
	return linkage->word;
}

const char * linkage_get_disjunct_str(const Linkage linkage, WordIdx w)
{
	Disjunct *dj;

	if (NULL == linkage) return "";
	if (NULL == linkage->disjunct_list_str)
	{
		lg_compute_disjunct_strings(linkage);
	}

	if (linkage->num_words <= w) return NULL; /* bounds-check */

	/* dj will be null if the word wasn't used in the parse. */
	dj = linkage->chosen_disjuncts[w];
	if (NULL == dj) return "";

	return linkage->disjunct_list_str[w];
}

double linkage_get_disjunct_cost(const Linkage linkage, WordIdx w)
{
	Disjunct *dj;

	if (linkage->num_words <= w) return 0.0; /* bounds-check */

	dj = linkage->chosen_disjuncts[w];

	/* dj may be null, if the word didn't participate in the parse. */
	if (dj) return dj->cost;
	return 0.0;
}

double linkage_get_disjunct_corpus_score(const Linkage linkage, WordIdx w)
{
	Disjunct *dj;

	if (linkage->num_words <= w) return 99.999; /* bounds-check */
	dj = linkage->chosen_disjuncts[w];

	/* dj may be null, if the word didn't participate in the parse. */
	if (NULL == dj) return 99.999;

	return lg_corpus_disjunct_score(linkage, w);
}

const char * linkage_get_word(const Linkage linkage, WordIdx w)
{
	if (!linkage) return NULL;
	if (linkage->num_words <= w) return NULL; /* bounds-check */
	return linkage->word[w];
}

int linkage_unused_word_cost(const Linkage linkage)
{
	/* The sat solver (currently) fails to fill in info */
	if (!linkage) return 0;
	return linkage->lifo.unused_word_cost;
}

double linkage_disjunct_cost(const Linkage linkage)
{
	/* The sat solver (currently) fails to fill in info */
	if (!linkage) return 0.0;
	return linkage->lifo.disjunct_cost;
}

int linkage_link_cost(const Linkage linkage)
{
	/* The sat solver (currently) fails to fill in info */
	if (!linkage) return 0;
	return linkage->lifo.link_cost;
}

double linkage_corpus_cost(const Linkage linkage)
{
	/* The sat solver (currently) fails to fill in info */
	if (!linkage) return 0.0;
	return linkage->lifo.corpus_cost;
}

/* =========== Get word sentence positions ============================== */

size_t linkage_get_word_byte_start(const Linkage linkage, WordIdx w)
{
	if (linkage->num_words <= w) return 0; /* bounds-check */
	return linkage->wg_path_display[w]->start - linkage->sent->orig_sentence;
}

size_t linkage_get_word_byte_end(const Linkage linkage, WordIdx w)
{
	if (linkage->num_words <= w) return 0; /* bounds-check */
	return linkage->wg_path_display[w]->end - linkage->sent->orig_sentence;
}

/* The character position is computed in a straightforward way, which may
 * not be efficient if more than one position is needed. If needed, it can
 * be changed to use caching of already-calculated positions. */

size_t linkage_get_word_char_start(const Linkage linkage, WordIdx w)
{
	if (linkage->num_words <= w) return 0; /* bounds-check */
	int pos = linkage->wg_path_display[w]->start - linkage->sent->orig_sentence;
	char *sentchunk = alloca(pos+1);
	strncpy(sentchunk, linkage->sent->orig_sentence, pos);
	sentchunk[pos] = '\0';
	return utf8_strlen(sentchunk);
}

size_t linkage_get_word_char_end(const Linkage linkage, WordIdx w)
{
	if (linkage->num_words <= w) return 0; /* bounds-check */
	int pos = linkage->wg_path_display[w]->end - linkage->sent->orig_sentence;
	char *sentchunk = alloca(pos+1);
	strncpy(sentchunk, linkage->sent->orig_sentence, pos);
	sentchunk[pos] = '\0';
	return utf8_strlen(sentchunk);
}
