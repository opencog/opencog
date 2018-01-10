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

/**
 * anysplit.c -- code that splits words into random morphemes.
 * This is used for the language-learning/morpheme-learning project.
 */

/* General assumptions:
 * - false is binary 0 (for memset())
 * - int is >= 32 bit (for random number)
 */

#include "utilities.h" /* included first, for MSVC rand_s() */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <externs.h>
#include <time.h>

#include "api-structures.h"
#include "dict-common/dict-affix.h"
#include "dict-common/dict-common.h"
#include "print/print-util.h" // For patch_subscript_mark()
#include "dict-common/regex-morph.h"
#include "error.h"
#include "tokenize.h"
#include "tok-structures.h"

#include "anysplit.h"


#define MAX_WORD_TO_SPLIT 31 /* in codepoins */

extern const char * const afdict_classname[];

typedef int p_start;     /* partition start in a word */
typedef p_start *p_list; /* list of partitions in a word */

typedef struct split_cache /* split cached by word length */
{
	size_t nsplits;      /* number of splits */
	p_list sp;           /* list of splits */
	bool *p_tried;       /* list of tried splits */
	bool *p_selected;    /* list of selected splits */
} split_cache;

typedef struct anysplit_params
{
	int nparts;                /* maximum number of suffixes to split to */
	size_t altsmin;            /* minimum number of alternatives to generate */
	size_t altsmax;            /* maximum number of alternatives to generate */
	Regex_node *regpre, *regmid, *regsuf; /* issue matching combinations  */
	split_cache scl[MAX_WORD_TO_SPLIT+1]; /* split cache according to word length */
} anysplit_params;

#define DEBUG_ANYSPLIT 0


#if DEBUG_ANYSPLIT
static const char *gw;
/* print the current partitions */
static void printsplit(int *ps, int n)
{
	static int sn = 0; /* split number */
	int pos = 0;
	int p;
	int l = strlen(gw);

	printf("split %d: ", sn++);
	for (pos = 0, p = 0; pos < l && p <= n; pos++)
	{
		if (pos == ps[p])
		{
			p++;
			putchar(' ');
		}
		putchar(gw[pos]);
	}
	putchar('\n');
}
static void printps(int *ps, int n)
{
	int i;

	printf("printps:");
	for (i = 0; i<=n; i++) printf(" ps[%d]=%d", i, ps[i]);
	printf("\n");
}
#endif

static void cache_partitions(p_list pl, int *ps, int p)
{
	memcpy(pl, ps, sizeof(p_start) * p);
}

	/* p = 5      */
	/*   0  1 2 3 */
	/*   |  | | | */
	/* 123456789  */
	/* l = 9      */
	/*            */
	/* n = 4      */
	/* ps[0] = 2  */
	/* ps[1] = 5  */
	/* ps[2] = 7  */
	/* ps[3] = 9  */

/**
 * `scl`: If NULL, return the index of the last split, else cache the
 * splits into scl.
 */
static int split_and_cache(int word_length, int nparts, split_cache *scl)
{

	int n;
	int maxindex;
	p_list ps = alloca(sizeof(p_start)*nparts); /* partition start */

	if (0 == word_length) return 0;

	/* The first partitioning is the whole word.
	 * (Using a first dummy partition would make this code unneeded.)
	 * But in any case the whole word here is unneeded, and I'm
	 * too lazy to change that.
	 */
	ps[0] = word_length;
	maxindex = 0;
	if (scl) cache_partitions(&scl->sp[0], ps, nparts);

	/* Generate all possible partitions up to nparts partitions */
	for (n = 1; n < nparts; n++)
	{
		/* increase the number of partitions */
		int m = 0;
		int t;

		ps[0] = 1;
		ps[n] = word_length; /* set last partition end (dummy partition start) */

		//printf("New number of partitions: n=%d\n", n);
		do
		{
			/* set next initial partitions lengths to 1 */
			//printf("Initialize: m=%d\n", m);
			for (t = m; t < n; t++)
			{
				ps[t] = ps[m] + (t-m);
				//printf("ps[%d]=%d ", t, ps[t]);
			}
			//printf("\n");

			/* move last partition */
			//printf("Moving m=%d ps[m]=%d ps[m+1]=%d\n", n-1, ps[n-1], ps[n]);
			for (m = n-1; ps[m] < ps[m+1]; ps[m]++)
			{
				maxindex++;
				if (scl) cache_partitions(&scl->sp[maxindex*nparts], ps, nparts);

#if DEBUG_ANYSPLIT
				printsplit(ps, n);
				printps(ps, n);
#endif
			}

			/* last partition got to size 1, backtrack */
			do
			{
				//printf("Backtrack m %d->%d\n", m, m-1);
				m--;
				/* continue as long as there is a place to move for partition m */
			} while (m >= 0 && ps[m] + 1 == ps[m+1]);
			if (m >= 0) ps[m]++;
		} while (m >= 0); /* we have still positions to move */
		//printf("End (n=%d)\n", n);
	}

	return maxindex+1;
}

void free_anysplit(Dictionary afdict)
{
	size_t i;
	anysplit_params *as = afdict->anysplit;

	if (NULL == as) return;

	for (i = 0; i < ARRAY_SIZE(as->scl); i++)
	{
		if (NULL == as->scl[i].sp) continue;
		free(as->scl[i].sp);
		free(as->scl[i].p_selected);
		free(as->scl[i].p_tried);
	}
	free_regexs(as->regpre);
	free_regexs(as->regmid);
	free_regexs(as->regsuf);
	free(as);
	afdict->anysplit = NULL;
}

/*
 * Returns: Number of splits.
 */
static int split(int word_length, int nparts, split_cache *scl)
{
	size_t nsplits;

	if (NULL == scl->sp)
	{
		nsplits = split_and_cache(word_length, nparts, NULL);
		//printf("nsplits %zu\n", nsplits);
		if (0 == nsplits)
		{
			prt_error("Error: nsplits=0 (word_length=%d, nparts=%d)\n",
				word_length, nparts);
			return 0;
		}
		scl->sp = malloc(sizeof(p_start)*nparts * nsplits);
		scl->p_selected = malloc(sizeof(*(scl->p_selected)) * nsplits);
		scl->p_tried = malloc(sizeof(*(scl->p_tried)) * nsplits);
		split_and_cache(word_length, nparts, scl);
		scl->nsplits = nsplits;
	}

	memset(scl->p_selected, false, sizeof(*(scl->p_selected)) * scl->nsplits);
	memset(scl->p_tried, false, sizeof(*(scl->p_tried)) * scl->nsplits);
	return scl->nsplits;
}

/**
 * Return a number between 0 and nsplits-1, including.
 * No need for a good randomness; mediocre randomness is enough.
 * We suppose int is 32 bit.
 */
static int rng_uniform(unsigned int *seedp, size_t nsplits)
{
	int res;

	res = rand_r(seedp);

	/* I don't mind the slight skew */
	return res % nsplits;

}

/* lutf is the length of the string, measured in code-points,
 * blen is the length of the string, measured in bytes.
 */
#define D_MM 7
static bool morpheme_match(Sentence sent,
	const char *word, size_t lutf, p_list pl)
{
	Dictionary afdict = sent->dict->affix_table;
	anysplit_params *as = afdict->anysplit;
	size_t bos = 0, cpos = 0; /* byte offset, code-point offset */
	int p;
	Regex_node *re;
	size_t blen = strlen(word);
	char *prefix_string = alloca(blen+1);

	lgdebug(+D_MM, "word=%s: ", word);
	for (p = 0; p < as->nparts; p++)
	{
		size_t b = utf8_strncpy(prefix_string, &word[bos], pl[p]-cpos);
		prefix_string[b] = '\0';
		bos += b;

		/* For flexibility, REGRPE is matched only to the prefix part,
		 * REGMID only to the middle suffixes, and REGSUF only to the
		 * suffix part - which cannot be the prefix. */
		if (0 == p) re = as->regpre;
		else if (pl[p] == (int) lutf) re = as->regsuf;
		else re = as->regmid;
		lgdebug(D_MM, "re=%s part%d=%s: ", re->name, p, prefix_string);

		/* A NULL regex always matches */
		if ((NULL != re) && (NULL == match_regex(re, prefix_string)))
		{
			lgdebug(D_MM, "No match\n");
			return false;
		}

		cpos = pl[p];
		if (cpos == lutf) break;
	}

	lgdebug(D_MM, "Match\n");
	return true;
}
#undef D_MM

static Regex_node * regbuild(const char **regstring, int n, int classnum)
{
	Regex_node *regex_root = NULL;
	Regex_node **tail = &regex_root; /* Last Regex_node in list */
	Regex_node *new_re;
	int i;

	for (i = 0; i < n; i++)
	{
		const char *r = regstring[i];

		/* Create a new Regex_node and add to the list. */
		new_re = malloc(sizeof(*new_re));
		new_re->name    = strdup(afdict_classname[classnum]);
		new_re->re      = NULL;
		new_re->next    = NULL;
		new_re->neg     = ('!' == r[0]);
		if (new_re->neg || (0 == strncmp(r, "\\!", 2))) r++;
		new_re->pattern = strdup(r);
		/* read_entry() (read-dict.c) invokes patch_subscript() also for the affix
		 * file. As a result, if a regex contains a dot it is patched by
		 * SUBSCRIPT_MARK. We undo it here. */
		patch_subscript_mark(new_re->pattern);

		*tail = new_re;
		tail = &new_re->next;
	}
	return regex_root;
}


/**
 * Affix classes:
 * REGPARTS  Max number of word partitions. Value 0 disables anysplit.
 * REGPRE    Regex for prefix
 * REGMID    Regex for middle suffixes
 * REGSUF    Regex for suffix
 * REGALTS   Number of alternatives to issue for a word.
 *           Two values: minimum and maximum.
 *           If the word has more possibilities to split than the minimum,
 *           but less then the maximum, then issue them unconditionally.
 */

/**
 * Initialize the anysplit parameter and cache structure.
 * Return true if initialization succeeded, or if dictionary does not use
 * anysplit (its not an error to not use anysplit!).  Return false if
 * init failed.
 */
#define D_AI 10
bool anysplit_init(Dictionary afdict)
{
	anysplit_params *as;
	size_t i;

	Afdict_class *regpre = AFCLASS(afdict, AFDICT_REGPRE);
	Afdict_class *regmid = AFCLASS(afdict, AFDICT_REGMID);
	Afdict_class *regsuf = AFCLASS(afdict, AFDICT_REGSUF);

	Afdict_class *regalts = AFCLASS(afdict, AFDICT_REGALTS);
	Afdict_class *regparts = AFCLASS(afdict, AFDICT_REGPARTS);

	if (0 == regparts->length)
	{
		if (verbosity_level(+D_AI))
			prt_error("Warning: File %s: Anysplit disabled (%s not defined)\n",
		             afdict->name, afdict_classname[AFDICT_REGPARTS]);
		return true;
	}
	if (1 != regparts->length)
	{
		prt_error("Error: File %s: Must have %s defined with one value\n",
		          afdict->name, afdict_classname[AFDICT_REGPARTS]);
		return false;
	}

	as = malloc(sizeof(anysplit_params));
	for (i = 0; i < ARRAY_SIZE(as->scl); i++) as->scl[i].sp = NULL;
	afdict->anysplit = as;

	as->regpre = regbuild(regpre->string, regpre->length, AFDICT_REGPRE);
	as->regmid = regbuild(regmid->string, regmid->length, AFDICT_REGMID);
	as->regsuf = regbuild(regsuf->string, regsuf->length, AFDICT_REGSUF);

	if (compile_regexs(as->regpre, NULL) != 0) return false;
	if (compile_regexs(as->regmid, NULL) != 0) return false;
	if (compile_regexs(as->regsuf, NULL) != 0) return false;

	as->nparts = atoi(regparts->string[0]);
	if (as->nparts < 0)
	{
		free_anysplit(afdict);
		prt_error("Error: File %s: Value of %s must be a non-negative number\n",
		          afdict->name, afdict_classname[AFDICT_REGPARTS]);
		return false;
	}
	if (0 == as->nparts)
	{
		free_anysplit(afdict);
		prt_error("Warning: File %s: Anysplit disabled (0: %s)\n",
		          afdict->name, afdict_classname[AFDICT_REGPARTS]);
		return true;
	}

	if (2 != regalts->length)
	{
		free_anysplit(afdict);
		prt_error("Error: File %s: Must have %s defined with 2 values\n",
		          afdict->name, afdict_classname[AFDICT_REGALTS]);
		return false;
	}
	as->altsmin = atoi(regalts->string[0]);
	as->altsmax = atoi(regalts->string[1]);
	if ((atoi(regalts->string[0]) <= 0) || (atoi(regalts->string[1]) <= 0))
	{
		free_anysplit(afdict);
		prt_error("Error: File %s: Value of %s must be 2 positive numbers\n",
		          afdict->name, afdict_classname[AFDICT_REGALTS]);
		return false;
	}

	return true;
}
#undef D_AI

/**
 * Split randomly.
 * Return true on success.
 * Return false when:
 * - disabled (i.e. when doing regular language processing).
 * - an error occurs (the behavior then is undefined).
 *   Such an error has not been observed yet.
 */
#define D_AS 5
bool anysplit(Sentence sent, Gword *unsplit_word)
{
	const char * word = unsplit_word->subword;
	Dictionary afdict = sent->dict->affix_table;
	anysplit_params *as;
	Afdict_class * stemsubscr;

	size_t l = strlen(word);
	size_t lutf = utf8_strlen(word);
	p_list pl;
	size_t bos, cpos; /* byte offset, codepoint offset */
	int p;
	int sample_point;
	size_t nsplits;
	size_t rndtried = 0;
	size_t rndissued = 0;
	size_t i;
	unsigned int seed = sent->rand_state;
	char *affix = alloca(l+2+1); /* word + ".=" + NUL: Max. affix length */
	bool use_sampling = true;

	if (NULL == afdict) return false;
	as = afdict->anysplit;

	if ((NULL == as) || (0 == as->nparts)) return false; /* Anysplit disabled */

	if (lutf > MAX_WORD_TO_SPLIT)
	{
		Gword *alt = issue_word_alternative(sent, unsplit_word, "AS>",
		                       0,NULL, 1,&word, 0,NULL);
		tokenization_done(sent, alt);
		return true;
	}

	if (0 == l)
	{
		prt_error("Warning: anysplit(): word length 0\n");
		return false;
	}

	stemsubscr = AFCLASS(afdict, AFDICT_STEMSUBSCR);

	// seed = time(NULL)+(unsigned int)(long)&seed;

#if DEBUG_ANYSPLIT
	gw = word;
#endif

	nsplits = split(lutf, as->nparts, &as->scl[lutf]);
	if (0 == nsplits)
	{
		prt_error("Warning: anysplit(): split() failed (shouldn't happen)\n");
		return false;
	}

	if (as->altsmax >= nsplits)
	{
		/* Issue everything */
		sample_point = -1;
		use_sampling = false;
	}

	lgdebug(+D_AS, "Start%s sampling: word=%s, nsplits=%zu, maxsplits=%d, "
	        "as->altsmin=%zu, as->altsmax=%zu\n", use_sampling ? "" : " no",
	        word, nsplits, as->nparts, as->altsmin, as->altsmax);

	while (rndtried < nsplits && (!use_sampling || (rndissued < as->altsmax)))
	{
		if (use_sampling)
		{
			sample_point = rng_uniform(&seed, nsplits);

			if (sample_point < 0) /* Cannot happen with rand_r() */
			{
				prt_error("Error: rng: %s\n", strerror(errno));
				return false;
			}
		}
		else
		{
			sample_point++;
		}

		lgdebug(D_AS, "Sample: %d ", sample_point);
		if (as->scl[lutf].p_tried[sample_point])
		{
			lgdebug(D_AS+1, "(repeated)\n");
			continue;
		}
		lgdebug(D_AS+1, "(new)");
		rndtried++;
		as->scl[lutf].p_tried[sample_point] = true;
		if (morpheme_match(sent, word, lutf, &as->scl[lutf].sp[sample_point*as->nparts]))
		{
			as->scl[lutf].p_selected[sample_point] = true;
			rndissued++;
		}
		else
		{
			lgdebug(D_AS, "\n");
		}
	}

	lgdebug(D_AS, "Results: word '%s' (byte-length=%zu utf-chars=%zu): %zu/%zu:\n",
	        word, lutf, l, rndissued, nsplits);

	for (i = 0; i < nsplits; i++)
	{
		const char **affixes = NULL;
		int num_sufixes;
		int num_affixes = 0;

		if (!as->scl[lutf].p_selected[i]) continue;

		pl = &as->scl[lutf].sp[i*as->nparts];
		bos = 0;
		cpos = 0;
		for (p = 0; p < as->nparts; p++)
		{
			size_t b = 0;
			if (pl[0] == (int)lutf)  /* This is the whole word */
			{
				b = utf8_strncpy(affix, &word[bos], pl[p]-cpos);
				affix[b] = '\0';
			}
			else
			if (0 == cpos)   /* The first, but not the only morpheme */
			{
				b = utf8_strncpy(affix, &word[bos], pl[p]-cpos);
				affix[b] = '\0';
			}
			else           /* 2nd and subsequent morphemes */
			{
				b = utf8_strncpy(affix, &word[bos], pl[p]-cpos);
				affix[b] = '\0';
				num_affixes++;
			}
			altappend(sent, &affixes, affix);

			bos += b;
			cpos = pl[p];
			// if (cpos == lutf) break; /* Same thing as below...*/
			if (bos == l) break;
		}

		const char **prefix_position, **stem_position , **suffix_position;
		switch (num_affixes)
		{
			case 0:
				prefix_position = NULL;
				stem_position = &affixes[0]; /* May be just a word here */
				suffix_position = NULL;
				num_sufixes = 0;
				break;
			case 1:
				prefix_position = NULL;
				stem_position = &affixes[0];
				suffix_position = &affixes[1];
				num_sufixes = 1;
				break;
			default:
				prefix_position =&affixes[0];
				stem_position = &affixes[1];
				suffix_position = &affixes[2];
				num_sufixes = num_affixes - 1;
				break;
		}
		if (num_affixes > 0)
		{
			if (0 != stemsubscr->length) {
				strcpy(affix, stem_position[0]);
				strcat(affix, stemsubscr->string[0]);
				stem_position[0] = affix;
			}
		}

		// XXX FIXME -- this is wrong - it assumes a
		// variable number of suffixes.
		/* Here a leading INFIX_MARK is added to the suffixes if needed. */
		Gword *alt = issue_word_alternative(sent, unsplit_word, "AS",
		        (NULL == prefix_position) ? 0 : 1, prefix_position,
		        1, stem_position,
		        num_sufixes, suffix_position);
		tokenization_done(sent, alt);
		free(affixes);
	}

	/* 0 == sent->rand_state denotes "repeatable rand". */
	if (0 != sent->rand_state) sent->rand_state = seed;
	return true;
}
#undef D_AS
