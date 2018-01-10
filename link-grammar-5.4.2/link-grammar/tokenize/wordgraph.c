/*************************************************************************/
/* Copyright (c) 2014 Amir Plivatsky                                     */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <stdlib.h>

#ifdef USE_WORDGRAPH_DISPLAY
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#ifdef HAVE_FORK
#include <unistd.h>    /* fork() and execl() */
#include <sys/wait.h>  /* waitpid() */
#endif
#ifdef HAVE_PRCTL
#include <sys/prctl.h> /* prctl() */
#endif
#include <signal.h>    /* SIG* */

#include <dict-common/dict-defines.h>  /* for SUBSCRIPT_MARK */
#include <print/print-util.h> /* for append_string */
#include <utilities.h> /* for dyn_str functions */
#endif /* USE_WORDGRAPH_DISPLAY */

#include "api-structures.h"
#include "error.h"
#include "string-set.h"
#include "tok-structures.h"
#include "wordgraph.h"

#ifdef __APPLE__
#define POPEN_DOT
#endif /* __APPLE__ */

/* === Gword utilities === */
/* Many more Gword utilities, that are used only in particular files,
 * are defined in these files statically. */

Gword *gword_new(Sentence sent, const char *s)
{
	Gword *gword = malloc(sizeof(*gword));

	memset(gword, 0, sizeof(*gword));
	assert(NULL != gword, "Null-string subword");
	gword->subword = string_set_add(s, sent->string_set);

	if (NULL != sent->last_word) sent->last_word->chain_next = gword;
	sent->last_word = gword;
	gword->node_num = sent->gword_node_num++;

	gword->gword_set_head = (gword_set){0};
	gword->gword_set_head.o_gword = gword;

	return gword;
}

/* FIXME: Remove it. */
Gword *empty_word(void)
{
	/*
	static Gword e = {
		.subword = EMPTY_WORD_MARK,
		.unsplit_word = &e,
		.morpheme_type = MT_EMPTY,
		.alternative_id = &e,
		.status = WS_INDICT,
	};
	*/
	return NULL;
}

static Gword **gwordlist_resize(Gword **arr, size_t len)
{
	arr = realloc(arr, (len+2) * sizeof(Gword *));
	arr[len+1] = NULL;
	return arr;
}

size_t gwordlist_len(const Gword **arr)
{
	size_t len = 0;
	if (arr)
		while (arr[len] != NULL) len++;
	return len;
}

void gwordlist_append(Gword ***arrp, Gword *p)
{
	size_t n = gwordlist_len((const Gword **)*arrp);

	*arrp = gwordlist_resize(*arrp, n);
	(*arrp)[n] = p;
}

#if 0
/**
 * Append a Gword list to a given Gword list (w/o duplicates).
 */
void gwordlist_append_list(const Gword ***to_word, const Gword **from_word)
{
	size_t to_word_arr_len = gwordlist_len(*to_word);

	for (const Gword **f = from_word; NULL != *f; f++)
	{
		size_t l;

		/* Note: Must use indexing because to_word may get realloc'ed. */
		for (l = 0; l < to_word_arr_len; l++)
			if (*f == (*to_word)[l]) break; /* Filter duplicates. */

		if (l == to_word_arr_len)
			gwordlist_append((Gword ***)to_word, (Gword *)*f);
	}
}

/**
 * Replace "count" words from the position "start" by word "wnew".
 */
static void wordlist_replace(Gword ***arrp, size_t start, size_t count,
                             const Gword *wnew)
{
	size_t n = gwordlist_len((const Gword **)(*arrp+start+count));

	memmove(*arrp+start+1, *arrp+start+count, (n+1) * sizeof(Gword *));
	(*arrp)[start] = (Gword *)wnew;
}
#endif

size_t wordgraph_pathpos_len(Wordgraph_pathpos *wp)
{
	size_t len = 0;
	if (wp)
		while (wp[len].word != NULL) len++;
	return len;
}

/**
 * `len` is the new length, not counting the terminating null entry.
 */
/* FIXME (efficiency): Initially allocate more than 2 elements */
Wordgraph_pathpos *wordgraph_pathpos_resize(Wordgraph_pathpos *wp,
                                            size_t len)
{
	wp = realloc(wp, (len+1) * sizeof(*wp));
	wp[len].word = NULL;
	return wp;
}

/**
 * Insert the gword into the path queue in reverse order of its hier_depth.
 *
 * The deepest wordgraph alternatives must be scanned first.
 * Otherwise, this sentence causes a flattening mess:
 * "T" this is a flattening test
 * (The mess depends on both "T" and "T matching EMOTICON, and any
 * 5 words after "T".)
 *
 * Parameters:
 *   same_word: mark that the same word is queued again.
 * For validation code only (until the wordgraph version is mature):
 *   used: mark that the word has already been issued into the 2D-array.
 *   diff_alternative: validate we don't queue words from the same alternative.
 */
bool wordgraph_pathpos_add(Wordgraph_pathpos **wp, Gword *p, bool used,
                              bool same_word, bool diff_alternative)
{
	size_t n = wordgraph_pathpos_len(*wp);
	Wordgraph_pathpos *wpt;
	size_t insert_here = n;

	assert(NULL != p);

#ifdef DEBUG
	if (verbosity_level(+9)) print_hier_position(p);
#endif

	if (NULL != *wp)
	{
		for (wpt = *wp; NULL != wpt->word; wpt++)
		{
			if (p == wpt->word)
				return false; /* already in the pathpos queue - nothing to do */

			/* Insert in reverse order of hier_depth. */
			if ((n == insert_here) && (p->hier_depth >= wpt->word->hier_depth))
				insert_here = wpt - *wp;

			/* Validate that there are no words in the pathpos queue from the same
			 * alternative. This can be commented out when the wordgraph code is
			 * mature. FIXME */
			if (diff_alternative)
			{
				assert(same_word||wpt->same_word||!in_same_alternative(p,wpt->word),
				       "wordgraph_pathpos_add(): "
				       "Word%zu '%s' is from same alternative of word%zu '%s'",
				       p->node_num, p->subword,
				       wpt->word->node_num, wpt->word->subword);
			}
		}
	}

	*wp = wordgraph_pathpos_resize(*wp, n+1);

	if (insert_here < n)
	{
		/* n+1 because n is the length of the array, not including the
		 * terminating null entry. We need to protect the terminating null.
		 */
		memmove(&(*wp)[insert_here+1], &(*wp)[insert_here],
		        (n+1 - insert_here) * sizeof (*wpt));
	}

	(*wp)[insert_here].word = p;
	(*wp)[insert_here].same_word = same_word;
	(*wp)[insert_here].used = used;
	(*wp)[insert_here].next_ok = false;

	return true;
}

/**
 *  Print linkage wordgraph path.
 */
void print_lwg_path(Gword **w, const char *title)
{
	lgdebug(+0, "%s: ", title);
	for (; *w; w++) lgdebug(0, "%s ", (*w)->subword);
	lgdebug(0, "\n");
}

#ifdef DEBUG
GNUC_UNUSED static const char *debug_show_subword(const Gword *w)
{
	return w->unsplit_word ? w->subword : "S";
}

GNUC_UNUSED void print_hier_position(const Gword *word)
{
	const Gword **p;

	err_msg(lg_Debug, "[Word %zu:%s hier_position(hier_depth=%zu): ",
	        word->node_num, word->subword, word->hier_depth);
	assert(2*word->hier_depth==gwordlist_len(word->hier_position), "word '%s'",
	       word->subword);

	for (p = word->hier_position; NULL != *p; p += 2)
	{
		err_msg(lg_Debug, "(%zu:%s/%zu:%s)",
		        p[0]->node_num, debug_show_subword(p[0]),
		        p[1]->node_num, debug_show_subword(p[1]));
	}
	err_msg(lg_Debug, "]\n");
}

/* Debug printout of a wordgraph Gword list. */
GNUC_UNUSED void gword_set_print(const gword_set *gs)
{
	printf("Gword list: ");

	if (NULL == gs)
	{
		printf("(null)\n");
		return;
	}

	for (; NULL != gs; gs = gs->next)
	{
		printf("word %p '%s' unsplit '%s'%s", gs->o_gword, (gs->o_gword)->subword,
		       (gs->o_gword)->unsplit_word->subword, NULL==gs->next ? "" : ", ");
	}
	printf("\n");

}
#endif

/**
 * Given a word, find its alternative ID.
 * An alternative is identified by a pointer to its first word, which is
 * getting set at the time the alternative is created at
 * issue_word_alternative(). (It could be any unique identifier - for coding
 * convenience it is a pointer.)
 *
 * Return the alternative_id of this alternative.
 */
static Gword *find_alternative(Gword *word)
{
	assert(NULL != word, "find_alternative(NULL)");
	assert(NULL != word->alternative_id, "find_alternative(%s): NULL id",
	       word->subword);

#if 0
	lgdebug(+0, "find_alternative(%s): '%s'\n",
	        word->subword, debug_show_subword(word->alternative_id));
#endif

	return word->alternative_id;
}

/**
 * Generate an hierarchy-position vector for the given word.
 * It consists of list of (unsplit_word, alternative_id) pairs, leading
 * to the word, starting from a sentence word. It is NULL terminated.
 * Original sentence words don't have any such pair.
 */
const Gword **wordgraph_hier_position(Gword *word)
{
	const Gword **hier_position; /* NULL terminated */
	size_t i = 0;
	Gword *w;
	bool is_leaf = true; /* the word is in the bottom of the hierarchy */

	if (NULL != word->hier_position) return word->hier_position; /* from cache */

	/*
	 * Compute the length of the hier_position vector.
	 */
	for (w = find_real_unsplit_word(word, true); NULL != w; w = w->unsplit_word)
		i++;
	if (0 == i) i = 1; /* Handle the dummy start/end words, just in case. */
	/* Original sentence words (i==1) have zero (i-1) elements. Each deeper
	 * unsplit word has an additional element. Each element takes 2 word pointers
	 * (first one the unsplit word, second one indicating the alternative in
	 * which it is found). The last +1 is for a terminating NULL. */
	word->hier_depth = i - 1;
	i = (2 * word->hier_depth)+1;
	hier_position = malloc(i * sizeof(*hier_position));

	/* Stuff the hierarchical position in a reverse order. */
	hier_position[--i] = NULL;
	w = word;
	while (0 != i)
	{
		hier_position[--i] = find_alternative(w);
		w = find_real_unsplit_word(w, is_leaf);
		hier_position[--i] = w;
		is_leaf = false;
	}

	word->hier_position = hier_position; /* cache it */
	return hier_position;
}

/**
 * Find if 2 words are in the same alternative of their common ancestor
 * unsplit_word.
 * "Same alternative" means at the direct alternative or any level below it.
 * A
 * |
 * +-B C D
 * |
 * +-E F
 *     |
 *     +-G H
 *     |
 *     +-I J
 * J and E (but not J and B) are in the same alternative of their common
 * ancestor unsplit_word A.
 * J and G are not in the same alternative (common ancestor unsplit_word F).
 *
 * Return true if they are, false otherwise.
 */
bool in_same_alternative(Gword *w1, Gword *w2)
{
	const Gword **hp1 = wordgraph_hier_position(w1);
	const Gword **hp2 = wordgraph_hier_position(w2);
	size_t i;

#if 0 /* DEBUG */
	print_hier_position(w1); print_hier_position(w2);
#endif

#if 0 /* BUG */
	/* The following is wrong!  Comparison to the hier_position of the
	 * termination word is actually needed when there are alternatives of
	 * different lengths at the end of a sentence.  This check then prevents
	 * the generation of empty words on the shorter alternative. */
	if ((NULL == w1->next) || (NULL == w2->next)) return false;/* termination */
#endif

	for (i = 0; (NULL != hp1[i]) && (NULL != hp2[i]); i++)
	{
		if (hp1[i] != hp2[i]) break;
	}

	/* In the even positions we have an unsplit_word.
	 * In the odd positions we have an alternative_id.
	 *
	 * If we are here when i is even, it means the preceding alternative_id was
	 * the same in the two words - so they belong to the same alternative.  If
	 * i is 0, it means these are sentence words, and sentence words are all in
	 * the same alternative (including the dummy termination word).
	 * If the hierarchy-position vectors are equal, i is also even, and words
	 * with equal hierarchy-position vectors are in the same alternative.
	 *
	 * If we are here when i is odd, it means the alternative_id at i is not
	 * the same in the given words, but their preceding unsplit_words are the
	 * same - so they clearly not in the same alternative.
	 */
	if (0 == i%2) return true;

	return false;
}

/**
 * Get the real unsplit word of the given word.
 * While the Wordgraph is getting constructed, when a subword has itself as one
 * of its own alternatives, it appears in the wordgraph only once, still
 * pointing to its original unsplit_word. It appears once in order not to
 * complicate the graph, and the unsplit_word is not changed in order not loss
 * information (all of these are implementation decisions). However, for the
 * hierarchy position of the word (when it is a word to be issued, i.e. a leaf
 * node) the real unsplit word is needed, which is the word itself. It is fine
 * since such a word cannot get split further.
 */
Gword *find_real_unsplit_word(Gword *word, bool is_leaf)
{
	/* For the terminating word, return something unique. */
	if (NULL == word->unsplit_word)
		return word;

	if (is_leaf && (word->status & WS_UNSPLIT))
		return word;

	return word->unsplit_word;
}

/* FIXME The following debug functions can be generated by a script running
 * from a Makefile and taking the values from structures.h, instead of hard
 * coding the strings as done here.  */

/**
 * Create a short form of flags summary for displaying in a word node.
 */
const char *gword_status(Sentence sent, const Gword *w)
{
	dyn_str *s = dyn_str_new();
	const char *r;
	size_t len;

	if (w->status & WS_UNKNOWN)
		dyn_strcat(s, "UNK|");
	if (w->status & WS_INDICT)
		dyn_strcat(s, "IN|");
	if (w->status & WS_REGEX)
		dyn_strcat(s, "RE|");
	if (w->status & WS_SPELL)
		dyn_strcat(s, "SP|");
	if (w->status & WS_RUNON)
		dyn_strcat(s, "RU|");
	if (w->status & WS_HASALT)
		dyn_strcat(s, "HA|");
	if (w->status & WS_UNSPLIT)
		dyn_strcat(s, "UNS|");
	if (w->status & WS_PL)
		dyn_strcat(s, "PL|");


	char *status_str = dyn_str_take(s);
	len = strlen(status_str);
	if (len > 0) status_str[len-1] = '\0'; /* ditch the last '|' */
	r = string_set_add(status_str, sent->string_set);
	free(status_str);
	return r;
}

#if USE_WORDGRAPH_DISPLAY || defined(DEBUG)
GNUC_UNUSED const char *gword_morpheme(Sentence sent, const Gword *w)
{
	const char *mt;
	char buff[64];

	switch (w->morpheme_type)
	{
		case MT_INVALID:
			mt = "MT_INVALID";
			break;
		case MT_WORD:
			mt = "MT_WORD";
			break;
		case MT_FEATURE:
			mt = "MT_FEATURE";
			break;
		case MT_INFRASTRUCTURE:
			mt = "MT_I-S";
			break;
		case MT_WALL:
			mt = "MT_WALL";
			break;
		case MT_EMPTY:
			mt = "MT_EMPTY";
			break;
		case MT_UNKNOWN:
			mt = "MT_UNKNOWN";
			break;
		case MT_TEMPLATE:
			mt = "MT_TEMPLATE";
			break;
		case MT_ROOT:
			mt = "MT_ROOT";
			break;
		case MT_CONTR:
			mt = "MT_CONTR";
			break;
		case MT_PUNC:
			mt = "MT_PUNC";
			break;
		case MT_STEM:
			mt = "MT_STEM";
			break;
		case MT_PREFIX:
			mt = "MT_PREFIX";
			break;
		case MT_MIDDLE:
			mt = "MT_MIDDLE";
			break;
		case MT_SUFFIX:
			mt = "MT_SUFFIX";
			break;
		default:
			/* No truncation is expected. */
			snprintf(buff, sizeof(buff), "MT_%d", w->morpheme_type);
			mt = string_set_add(buff, sent->string_set);
	}

	return mt;
}
#endif /* USE_WORDGRAPH_DISPLAY || defined(DEBUG) */

#if USE_WORDGRAPH_DISPLAY
/* === Wordgraph graphical representation === */

static void wordgraph_legend(dyn_str *wgd, unsigned int mode)
{
	size_t i;
	static char const *wst[] = {
		"RE", "Matched a regex",
		"SP", "Result of spell guess",
		"RU", "Separated run-on word",
		"HA", "Has an alternative",
		"UNS", "Also unsplit_word",
		"IN", "In the dict file",
		"FI", "First char is uppercase"
	};

	append_string(wgd,
		"subgraph cluster_legend {\n"
		"label=Legend;\n"
		"%s"
		"legend [label=\"subword\\n(status-flags)\\nmorpheme-type\"];\n"
		"legend [xlabel=\"ordinal-number\\ndebug-label\"];\n"
		"%s"
		"legend_width [width=4.5 height=0 shape=none label=<\n"
		"<table border='0' cellborder='1' cellspacing='0'>\n"
		"<tr><td colspan='2'>status-flags</td></tr>\n",
		(mode & WGR_SUB) ? "subgraph cluster_unsplit_word {\n"
		                   "label=\"ordinal-number unsplit-word\";\n" : "",
		(mode & WGR_SUB) ? "}\n" : ""

	);
	for (i = 0; i < sizeof(wst)/sizeof(wst[0]); i += 2)
	{
		append_string(wgd,
		  "<tr><td align='left'>%s</td><td align='left'>%s</td></tr>\n",
		  wst[i], wst[i+1]);
	}

	append_string(wgd,
		"</table>>];"
		"}\n"
		"subgraph cluster_legend_top_space {\n"
			"style=invis legend_dummy [style=invis height=0 shape=box]\n"
		"};\n"
	);
}

/**
 * Graph node name: Add "Sentence:" for the main node; Convert SUBSCRIPT_MARK.
 * Also escape " and \ with a \.
 */
static const char *wlabel(Sentence sent, const Gword *w)
{
	const char *s;
	const char sentence_label[] = "Sentence:\\n";
	dyn_str *l = dyn_str_new();
	char c0[] = "\0\0";

	assert((NULL != w) && (NULL != w->subword), "Word must exist");
	if ('\0' == *w->subword)
		 return string_set_add("(nothing)", sent->string_set);

	if (w == sent->wordgraph) dyn_strcat(l, sentence_label);

	for (s = w->subword; *s; s++)
	{
		switch (*s)
		{
			case SUBSCRIPT_MARK:
				dyn_strcat(l, ".");
				break;
			case '\"':
				dyn_strcat(l, "\\\"");
				break;
			case '\\':
				dyn_strcat(l, "\\");
				break;
			default:
				*c0 = *s;
				dyn_strcat(l, c0);
		}
	}

	char *label_str = dyn_str_take(l);
	s = string_set_add(label_str, sent->string_set);
	free(label_str);
	return s;
}

/**
 *  Generate the wordgraph in dot(1) format, for debug.
 */
static dyn_str *wordgraph2dot(Sentence sent, unsigned int mode, const char *modestr)
{
	const Gword *w;
	Gword	**wp;
	dyn_str *wgd = dyn_str_new(); /* the wordgraph in dot representation */
	char nn[2*sizeof(char *) + 2 + 2 + 1]; /* \"%p\" node name: "0x..."+NUL*/

	append_string(wgd, "# Mode: %s\n", modestr);
	dyn_strcat(wgd, "digraph G {\nsize =\"30,20\";\nrankdir=LR;\n");
	if ((mode & (WGR_SUB)) && !(mode & WGR_COMPACT))
		dyn_strcat(wgd, "newrank=true;\n");
	if (mode & WGR_LEGEND) wordgraph_legend(wgd, mode);
	append_string(wgd, "\"%p\" [shape=box,style=filled,color=\".7 .3 1.0\"];\n",
	              sent->wordgraph);

	for (w = sent->wordgraph; w; w = w->chain_next)
	{
		bool show_node;

		if (!(mode & WGR_UNSPLIT) && (MT_INFRASTRUCTURE != w->morpheme_type))
		{
			Gword *wu;

			show_node = false;
			/* In this mode nodes that are only unsplit_word are not shown. */
			for (wu = sent->wordgraph; wu; wu = wu->chain_next)
			{
				if (NULL != wu->next)
				{
					for (wp = wu->next; *wp; wp++)
					{
						if (w == *wp)
						{
							show_node = true;
							break;
						}
					}
				}
			}

			if (!show_node) continue;
		}

		snprintf(nn, sizeof(nn), "\"%p\"", w);

		/* Subword node format:
		 *                     +------------------+
		 *                     +                  +
		 *                     +    w->subword    +
		 *                     +    (w->flags)    +
		 *                     + w->morpheme_type +
		 *                     +                  +
		 *                     +------------------+
		 *          w->node_num  } <- external node label
		 *           w->label    }
		 *
		 * The flags and morpheme type are printed symbolically.
		 * The node_num field is the ordinal number of word creation.
		 * The label shows the code positions that created the subword.
		 * The external node label may appear at other positions near the node.
		 *
		 * FIXME: Use HTML labels.
		 */

		append_string(wgd, "%s [label=\"%s\\n(%s)\\n%s\"];\n", nn,
			wlabel(sent, w), gword_status(sent, w), gword_morpheme(sent, w));

		if (!(mode & WGR_DBGLABEL))
		{
			append_string(wgd, "%s [xlabel=\"%zu",
							  nn, w->node_num);
		}
		else
		{
			append_string(wgd, "%s [xlabel=\"%zu\\n%s",
							  nn, w->node_num, w->label);
		}

		/* For debugging this function: display also hex node names. */
		if (mode & WGR_DOTDEBUG)
			append_string(wgd, "\\n%p-%s", w, wlabel(sent, w));

		dyn_strcat(wgd, "\"];\n");

		if (NULL != w->next)
		{
			for (wp = w->next; *wp; wp++)
			{
				append_string(wgd, "%s->\"%p\" [label=next color=red];\n",
				              nn, *wp);
			}
		}
		if (mode & WGR_PREV)
		{
			if (NULL != w->prev)
			{
				for (wp = w->prev; *wp; wp++)
				{
					append_string(wgd, "%s->\"%p\" [label=prev color=blue];\n",
					              nn, *wp);
				}
			}
		}
		if (mode & WGR_UNSPLIT)
		{
			if (!(mode & WGR_SUB) && (NULL != w->unsplit_word))
			{
				append_string(wgd, "%s->\"%p\" [label=unsplit];\n",
				              nn, w->unsplit_word);
			}
		}
	}

	if (mode & WGR_SUB)
	{
		const Gword *old_unsplit = NULL;

		for (w = sent->wordgraph; w; w = w->chain_next)
		{
			if (NULL != w->unsplit_word)
			{
				if (w->unsplit_word != old_unsplit)
				{
					if (NULL != old_unsplit) dyn_strcat(wgd, "}\n");
					append_string(wgd, "subgraph \"cluster-%p\" {", w->unsplit_word);
					append_string(wgd, "label=\"%zu %s\"; \n",
						w->unsplit_word->node_num, wlabel(sent, w->unsplit_word));

					old_unsplit = w->unsplit_word;
				}
				snprintf(nn, sizeof(nn), "\"%p\"", w);
				if (strstr(dyn_str_value(wgd), nn))
					append_string(wgd, "\"%p\"; ", w);
			}
		}
		dyn_strcat(wgd, "}\n");
	}
	else
	{
#ifdef WGR_SHOW_TERMINATOR_AT_LHS /* not defined - not useful */
		const Gword *terminating_node = NULL;
#endif

		dyn_strcat(wgd, "{rank=same; ");
		for (w = sent->wordgraph->chain_next; w; w = w->chain_next)
		{
			snprintf(nn, sizeof(nn), "\"%p\"", w);
			if (IS_SENTENCE_WORD(sent, w) &&
			    ((mode & WGR_UNSPLIT) || strstr(dyn_str_value(wgd), nn)))
			{
				append_string(wgd, "%s; ", nn);
			}

#ifdef WGR_SHOW_TERMINATOR_AT_LHS
			if (NULL == w->next) terminating_node = w;
#endif
		}
		dyn_strcat(wgd, "}\n");

#ifdef WGR_SHOW_TERMINATOR_AT_LHS
		if (terminating_node)
			append_string(wgd, "{rank=sink; \"%p\"}\n", terminating_node);
#endif
	}

	dyn_strcat(wgd, "\n}\n");

	return wgd;
}

#if defined(HAVE_FORK) && !defined(POPEN_DOT)
static pid_t pid; /* XXX not reentrant */

#ifndef HAVE_PRCTL
/**
 * Cancel the wordgraph viewers, to be used if there is fork() but no prctl().
 */
static void wordgraph_show_cancel(void)
{
		kill(pid, SIGTERM);
}
#endif /* HAVE_FORK */
#endif /* HAVE_PRCTL */

#ifndef DOT_COMMNAD
#define DOT_COMMAND "dot"
#endif

#ifndef DOT_DRIVER
#define DOT_DRIVER "-Txlib"
#endif

/* In case files are used, their names are fixed. So more than one thread
 * (or program) cannot use the word-graph display at the same time. This
 * can be corrected, even though there is no much point to do that
 * (displaying the word-graph is for debug). */
#define DOT_FILENAME "lg-wg.vg"

#define POPEN_DOT_CMD DOT_COMMAND" "DOT_DRIVER
#ifndef POPEN_DOT_CMD_NATIVE
#  ifdef _WIN32
#    ifndef IMAGE_VIEWER
#      define IMAGE_VIEWER "rundll32 PhotoViewer,ImageView_Fullscreen"
#    endif
#    define WGJPG "%TEMP%\\lg-wg.jpg"
#    define POPEN_DOT_CMD_NATIVE \
				DOT_COMMAND" -Tjpg>"WGJPG"&"IMAGE_VIEWER" "WGJPG"&del "WGJPG
#  elif __APPLE__
#    ifndef IMAGE_VIEWER
#      define IMAGE_VIEWER "open -W"
#    endif
#    define WGJPG "$TMPDIR/lg-wg.jpg"
#    define POPEN_DOT_CMD_NATIVE \
				DOT_COMMAND" -Tjpg>"WGJPG";"IMAGE_VIEWER" "WGJPG";rm "WGJPG
#  else
#    define POPEN_DOT_CMD_NATIVE POPEN_DOT_CMD
#  endif
#endif

#if !defined HAVE_FORK || defined POPEN_DOT
#ifdef _MSC_VER
#define popen _popen
#define pclose _pclose
#endif
/**
 * popen a command with the given input.
 * If the system doesn't have fork(), popen() is used to launch "dot".
 * This is an inferior implementation than the one below that uses
 * fork(), in which the window remains open and is updated automatically
 * when new sentences are entered. With popen(), the program blocks at
 * pclose() and the user needs to close the window after each sentence.
 */
static void x_popen(const char *cmd, const char *wgds)
{
	FILE *const cmdf = popen(cmd, "w");

	if (NULL == cmdf)
	{
		prt_error("Error: popen of '%s' failed: %s\n", cmd, strerror(errno));
	}
	else
	{
		if (fprintf(cmdf, "%s", wgds) == -1)
			prt_error("Error: print to display command: %s\n", strerror(errno));
		if (pclose(cmdf) == -1)
			prt_error("Error: pclose of display command: %s\n", strerror(errno));
	}
}
#else
static void x_forkexec(const char *const argv[], pid_t *vpid)
{
	/* Fork/exec a graph viewer, and leave it in the background until we exit.
	 * On exit, send SIGHUP. If prctl() is not available and the program
	 * crashes, then it is left to the user to exit the viewer. */
	if (0 < *vpid)
	{
		pid_t rpid = waitpid(*vpid, NULL, WNOHANG);

		if (0 == rpid) return; /* viewer still active */
		if (-1 == rpid)
		{
			prt_error("Error: waitpid(%d): %s\n", *vpid, strerror(errno));
			*vpid = 0;
			return;
		}
	}

	*vpid = fork();
	switch (*vpid)
	{
		case -1:
			prt_error("Error: fork(): %s\n", strerror(errno));
			break;
		case 0:
#ifdef HAVE_PRCTL
			if (-1 == prctl(PR_SET_PDEATHSIG, SIGHUP))
					prt_error("Error: prctl: %s\n", strerror(errno));
#endif
			/* Not closing fd 0/1/2, to allow interaction with the program */
			execvp(argv[0], (char **)argv);
			prt_error("Error: execlp of %s: %s\n", argv[0], strerror(errno));
			_exit(1);
		default:
#ifndef HAVE_PRCTL
			if (0 != atexit(wordgraph_show_cancel))
				 prt_error("Warning: atexit(wordgraph_show_cancel) failed.\n");
#endif
			break;
	}
}
#endif /* !defined HAVE_FORK || defined POPEN_DOT */

#ifdef _WIN32
#define TMPDIR (getenv("TEMP") ? getenv("TEMP") : ".")
#else
#define TMPDIR (getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp")
#endif

#define concatfn(fn, fn1, fn2) \
	(fn=alloca(strlen(fn1)+strlen(fn2)+2),\
	 strcpy(fn, fn1), strcat(fn, "/"), strcat(fn, fn2))

static void wordgraph_unlink_xtmpfile(void)
{
	char *fn;

	if (!test_enabled("gvfile"))
	{
		concatfn(fn, TMPDIR, DOT_FILENAME);
		if (unlink(fn) == -1)
			prt_error("Warning: Cannot unlink %s: %s\n", fn, strerror(errno));
	}
}

/**
 * Display the word-graph in the indicated mode.
 * This is for debug. It is not reentrant due to the static pid and the
 * possibly created fixed filenames.
 * When Using X11, a "dot -Txlib" program is launched on the graph
 * description file.  The xlib driver refreshes the graph when the file is
 * changed, displaying additional sentences in the same window.  The viewer
 * program exits on program end (see the comments in the code).  When
 * compiled with MSVC or MINGW, the system PhotoViewer is used by default,
 * unless !wg:x is used (for using X11 when available).
 *
 * The "dot" and the "PhotoViewer" programs must be in the PATH.
 *
 * FIXME? "dot" may get a SEGV due to memory corruptions in it (a known
 * problem - exists even in 2.38). This can be worked-around by trying it
 * again until it succeeds (but the window size, if changed by the user,
 * will not be preserved).
 *
 * modestr: a graph display mode as defined in wordgraph.h (default "ldu").
 */
void wordgraph_show(Sentence sent, const char *modestr)
{
	dyn_str *wgd;
	char *gvf_name = NULL;
	bool generate_gvfile = test_enabled("gvfile"); /* keep it for debug */
	char *wgds;
	bool gvfile = false;
	uint32_t mode = 0;
	const char *mp;

	for (mp = modestr; '\0' != *mp && ',' != *mp; mp++)
	{
		if ((*mp >= 'a') && (*mp <= 'z')) mode |= 1<<(*mp-'a');
	}
	if ((0 == mode) || (WGR_X11 == mode))
		mode |= WGR_LEGEND|WGR_DBGLABEL|WGR_UNSPLIT;

	wgd = wordgraph2dot(sent, mode, modestr);
	wgds = dyn_str_take(wgd);

#if defined(HAVE_FORK) && !defined(POPEN_DOT)
	gvfile = true;
#endif

	if (gvfile || generate_gvfile)
	{
		FILE *gvf;
		bool gvf_error = false;
		static bool wordgraph_unlink_xtmpfile_needed = true;

		concatfn(gvf_name, TMPDIR, DOT_FILENAME);
		gvf = fopen(gvf_name, "w");
		if (NULL == gvf)
		{
			prt_error("Error: wordgraph_show: open %s failed: %s\n",
						 gvf_name, strerror(errno));
		}
		else
		{
			if (fprintf(gvf, "%s", wgds) == -1)
			{
				gvf_error = true;
				prt_error("Error: wordgraph_show: print to %s failed: %s\n",
							 gvf_name, strerror(errno));
			}
			if (fclose(gvf) == EOF)
			{
				gvf_error = true;
				prt_error("Error: wordgraph_show: close %s failed: %s\n",
							  gvf_name, strerror(errno));
			}
		}
		if (gvf_error && gvfile) /* we need it - cannot continue */
		{
			free(wgds);
			return;
		}

		if (wordgraph_unlink_xtmpfile_needed)
		{
			/* The filename is fixed - removal needed only once. */
			wordgraph_unlink_xtmpfile_needed = false;
			atexit(wordgraph_unlink_xtmpfile);
		}
	}

#ifdef _WIN32
#define EXITKEY "ALT-F4"
#elif __APPLE__
#define EXITKEY "⌘-Q"
#endif

#ifdef EXITKEY
	prt_error("Press "EXITKEY" in the graphical display window to continue\n");
#endif

#if !defined HAVE_FORK || defined POPEN_DOT
	x_popen((mode & WGR_X11)? POPEN_DOT_CMD : POPEN_DOT_CMD_NATIVE, wgds);
#else
	{
		assert(NULL != gvf_name, "DOT filename not initialized (#define mess?)");
		const char *const args[] = { DOT_COMMAND, DOT_DRIVER, gvf_name, NULL };
		x_forkexec(args, &pid);
	}
#endif
	free(wgds);
}
#else
void wordgraph_show(Sentence sent, const char *modestr)
{
		prt_error("Error: Not configured with --enable-wordgraph-display\n");
}
#endif /* USE_WORDGRAPH_DISPLAY */
