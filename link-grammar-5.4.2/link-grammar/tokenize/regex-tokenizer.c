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

/* FIXME: Fold long lines. */

#ifdef USE_REGEX_TOKENIZER

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <externs.h>
#include <time.h>

#include "regex-tokenizer.h"
#include "dict-api.h"
#include "dict-common.h"
#include "error.h"
#include "regex-morph.h"
#include "structures.h"
#include "tokenize.h"
#include "utilities.h"
#include "word-utils.h"
#include "dict-file/read-dict.h"

#include "pcre.h"

/* Tokenizer flags. */
#define MARK_TOKENS 0x1	/* stem/affix marks in the result tokens (not implemented) */

/* Debug signature for the initial end of sub-pattern 0 (subp[0].e).  If we find
 * it while printing an alternative, it means we have a logic failure, because
 * we should not reach the end of the pattern without a match which assigns
 * there the matched sub-pattern string end. */
#define SUBP0END_DEBUG_SIGNATURE -2

//extern const char const * afdict_classname[];

typedef enum
{
	CALLBACK_REP,
	CALLBACK_END,
	CALLBACK_CONSTANT_START,   /* UNUSED */
	CALLBACK_CONSTANT_END      /* UNUSED */
} callback_num;

typedef struct ov
{
	int s;
	int e;
} ov;

#ifdef REGEX_TOKENIZER_CACHE
/* TODO */
/* Match cache bit vector. */
typedef struct bitvec
{
	int len;  /* current vector length, in bytes */
	char *vec;
	int get;  /* cache get counter */
	int set;  /* cache set counter */
} bitvec;
#endif

/* info per capture group number */
typedef struct  cgnum
{
		Dictionary dict;       /* dictionary to use */
		const char *afclass;   /* affix class, or NULL for main dict */
		const char *lookup_mark; /* potential stem or infix marks */
		char lookup_mark_pos;  /* "+" (append) or "-" (prepend) */
		const char *name;      /* currently only for result tagging printout */
#ifdef REGEX_TOKENIZER_CACHE /* TODO */
		bitvec *mcache;        /* substring match cache */
#endif
		/* FIXME: Maybe add formatting function for SUF, PRE, STEM */
} cgnum;

#define MAX_SUBP 100
typedef struct callout_data
{
	int function;             /* callout function multiplexing */
	const char *pattern;
	int test;
	ov subp[MAX_SUBP];        /* sub-pattern array */
	int capture_level[MAX_SUBP];
	int subp_i;             /* current sub-pattern index */
	bool subp_ovfl;         /* subp array overflow */
	int capture_last;			/* UNUSED */
	const char ***wordlist;
	cgnum **cgnum;
	// bool is_constant; /* a constant alternation - don't lookup (FIXME. UNUSED)*/
	int alt_counter;  /* counter for number of alternatives */
} callout_data;

/**
 * Get a regex (of 4.0.regex) by name.
 * Replace all capturing groups by non-capturing ones, since the invoking
 * function cannot currently handle them. Hence back references are not
 * supported. This can be fixed if needed.
 *
 * If a regex name appears multiple times, concatenate them using an alternation
 * bar. Remove anchors ^ and $ if exist (suppose they can only appear at the
 * start and end of the regex, as currently in 4.0.regex).
 */
static char *get_regex_by_name(Dictionary const dict, const char * const name)
{
	dyn_str * const pat = dyn_str_new();
	char *result = NULL;
	Regex_node *re = dict->regex_root;
	const char *p;

	while (NULL != re)
	{
		if (0 == strcmp(re->name, name))
		{
			/* re analyze state */
			bool insqb = false;        /* in square brackets */
			bool qn = false;           /* quote next character */

			p = re->pattern;
			if ('\0' != pat->str[0]) dyn_strcat(pat, "|");
			if ('^' == *p) p++;

			/* Change groups in POSIX regex to PCRE non-capturing groups.
			 * FIXME: Add support for PCRE syntax,
			 * especially, skip (?...) and (*...).
			 * The following code supports backslash and square brackets.
			 * It supposes the regex is valid. */
			for (; '\0' != *p; p++)
			{
				char c0[2] = "\0\0";

				if (qn)
				{
					qn = false;
				}
				else
				{
					switch (*p)
					{
						case '\\':
							qn = true;
							break;
						case '[':
							insqb = true;
							break;
						case ']':
							if (p > re->pattern && '[' == p[-1]) break;
							insqb = false;
							break;
						case '(':
							if (insqb) break;
							dyn_strcat(pat, "(?:");
							continue;
					}
				}
				if ('$' != *p || '\0' != p[1])
				{
					c0[0] = *p;
					dyn_strcat(pat, c0);
				}
			}
		}
		re = re->next;
	}

	if ('\0' != pat->str[0]) result = strdup(pat->str);
	dyn_str_delete(pat);
	return result;
}

static void printov(const char *str, ov *ov, int top, callout_data *cd, bool is_pcreov)
{
	int i;
	const cgnum *cgnump = NULL;

	for (i = 0; i < top; i++)
	{
		printf("%2d", i);
		if (!is_pcreov && (NULL != cd) && (NULL != cd->capture_level))
			printf(" (%d)", (ov[i].e < 0) ? 0 : cd->capture_level[i]);
		printf(": ");
		if (ov[i].s < 0)
		{
			printf(" <unset>");
		} else
		{
			if (ov[i].e < 0)
				printf(" END<0 (%d,%d)", ov[i].s, ov[i].e);
			else
				printf(" %.*s (%d,%d)", ov[i].e - ov[i].s, str + ov[i].s, ov[i].s, ov[i].e);
		}

		/* Find the tokenizer capture group info for the current OV element:
		 * - For PCRE OV, use its index (if > 0) as capture group.
		 * - For the tokenizer OV, use the recorded capture level.
		 *  Since the cgnum array is 0-based and the first parenthesized capture
		 *  group is 1, subtract 1 to get the actual index. */
		if ((NULL != cd) && (NULL != cd->capture_level) && (NULL != cd->cgnum) &&
				(!is_pcreov || (i > 0)) && ov[i].e >= 0)
			cgnump = cd->cgnum[(is_pcreov ? i : cd->capture_level[i]) - 1];

		if (NULL != cgnump)
		{
			const char *a = "", *p = "";
			char lookup_mark[10];
			char *sm;

			if (NULL != cgnump->lookup_mark)
			{
				if ('a' == cgnump->lookup_mark_pos)
				{
					safe_strcpy(lookup_mark, cgnump->lookup_mark, sizeof(lookup_mark));
					sm = strrchr(lookup_mark, SUBSCRIPT_MARK);
					if (NULL != sm) *sm = '.';
					a = lookup_mark;
				}
				else
				{
					p = cgnump->lookup_mark;
				}
			}
			printf(" [%s%s%s]", p, cgnump->name, a);
		}

		printf("\n");
	}
}

/**
 * Compare a portion of the tokenized string, starting at word_stat with length
 * of numchar, to the dictionary or affix class word that is defined in the
 * capture group whose info is pointed to by cgnump.
 *
 * FIXME: Return int instead of bool, see the comment at E1 below.
 */
static bool is_word(const char *word_start, int numchar, cgnum *cgnump)
{
	Dictionary const dict = cgnump->dict;
	const char * const afclass = cgnump->afclass;
	const int lookup_mark_len =
		(NULL != cgnump->lookup_mark) ? strlen(cgnump->lookup_mark) : 0;
	char * const word = alloca(numchar+lookup_mark_len+1);
#ifdef AFFIX_DICTIONARY_TREE
	const Dict_node *dn;
#endif
	const Afdict_class *ac;
	size_t i;

	/* Append/prepend stem/infix marks. */
	if (NULL == cgnump->lookup_mark)
	{
		strncpy(word, word_start, numchar);
		word[numchar] = '\0';
	}
	else
	{
		switch (cgnump->lookup_mark_pos)
		{
		case 'p': /* prepend a mark */
			strcpy(word, cgnump->lookup_mark);
			strncat(word, word_start, numchar);
			word[numchar+lookup_mark_len] = '\0';
			break;
		case 'a': /* append a mark */
			strncpy(word, word_start, numchar);
			strcpy(word+numchar, cgnump->lookup_mark);
			break;
		default:
			printf("is_word:E3('%x' %s)", cgnump->lookup_mark_pos, cgnump->lookup_mark);
			strncpy(word, word_start, numchar);
			word[numchar] = '\0';
		}
	}

	lgdebug(7, "LOOKUP '%s' in %s: ", word, dict->name);
	if (0 == afclass) return boolean_dictionary_lookup(dict, word);

	/* We don't have for now a tree representation of the affix file, only lists */
#ifdef AFFIX_DICTIONARY_TREE
	dn = lookup_list(dict, word);
	printf("WORD %s afclass %s dn %p\n", word, afclass, dn);
	if (NULL == dn) return false;

	for (; NULL != dn; dn = dn->left)
	{
		const char *con = word_only_connector(dn);
		if (NULL == con)
		{
			/* Internal error - nothing else to do for now unless we don't
			 * rerun bool, but return an int so -1 signifies an error. */
			printf("is_word(%s):E1 ", word);
		}
		printf("CON '%s'\n", con);
		if (0 == strcmp(afclass, con)) return true;
	}
#else
		/* Make it the hard way. */
		ac = afdict_find(dict, afclass, /*notify_err*/false);
		if (NULL == ac)
		{
			/* Internal error - nothing else to do for now unless we don't
			 * rerun bool, but return an int so -1 signifies an error. */
			printf("is_word(%s):E2 ", word);
		}

		for (i = 0; i < ac->length; i++)
		{
			if (0 == strcmp(ac->string[i], word)) return true;
		}
#endif

	return false;
}

static int callout(pcre_callout_block *cb)
{
	callout_data *cd = cb->callout_data;
	ov *cb_ov = (ov *)&cb->offset_vector[2*cb->capture_last];

#if 0
	const char **wordlist = NULL;
#endif
	cgnum *cgnum = NULL;
	const char *openp;
	const char *endname;
	bool subp_updated = false;

	if ((NULL != cd->cgnum) && (-1 != cb->capture_last))
	{
		cgnum = cd->cgnum[cb->capture_last-1];
	}
	lgdebug(6, "Callout %d: capture_last %d cgnum %p\n",
	        cb->callout_number, cb->capture_last, cgnum);

	if (verbosity >= 6)
		printov(cb->subject, (ov *)cb->offset_vector, cb->capture_top, cd, /*is_pcreov*/true);

	switch(cb->callout_number)
	{
	case CALLBACK_REP:
		if (cb->capture_last > 0)
		{
			int subp_i = cd->subp_i;
			ov *subp = &cd->subp[subp_i];

			lgdebug(2, "Current capture %d: s=%d, e=%d\n",
			        cb->capture_last, cb_ov->s, cb_ov->e);
			assert(cb_ov->s>=0 && cb_ov->e>=0, "Bad start/end in capture group %d: s=%d e=%d",
			       cb->capture_last, cb_ov->s, cb_ov->e);

			if (verbosity >= 6)
			{
				printf("INITIAL subp:\n");
				if (cd->subp_ovfl) printf("OVERFLOW\n"); /* shouldn't happen */
				printov(cb->subject, cd->subp, cd->subp_i+1, cd, /*is_pcreov*/false);
			}

			/* Record all the captures into the subp (sub-pattern) vector.
			 * If we capture a continuation to another capture then it is a new
			 * capture. Else we update a previous position in subp. There should be
			 * no gaps between the capture strings.
			 * FIXME: Handled null matches properly. Need to use cd->capture_level
			 * to remember at which level a null match has been captured.
			 * FIXME: Move after the word lookup (efficiency).
			 * FIXME: Increment subp instead of cd->subp_i (cosmetic fix). */

			if (cb_ov->s > subp->s)
			{
				if (cb_ov->s == subp->e)
				{
					cd->subp_i++;
					if (cd->subp_i == MAX_SUBP)
					{
						cd->subp_ovfl = true;
						return PCRE_ERROR_CALLOUT;
					}
					lgdebug(2, "OV start gt, update next sub-pattern %d\n", cd->subp_i);
					cd->subp[cd->subp_i] = *cb_ov;
					subp_updated = true;
				}
				else
				{
					printf("Capture group %d (s=%d e=%d) makes a hole (subp_i %d: s=%d e=%d)\n",
							 cb->capture_last, subp->s, subp->e, subp_i, cb_ov->s, cb_ov->e);
					return PCRE_ERROR_CALLOUT;
				}
			}
			else
			{
				/* A backtrack occurred. */
				for (subp_i = cd->subp_i; subp_i >= 0; subp_i--)
				{
					subp = &cd->subp[subp_i];

					lgdebug(2, "Checking recorded sub-pattern %d: s=%d e=%d: ",
							  subp_i, subp->s,  subp->e);

					if (cb_ov->s == subp->s)
					{
						lgdebug(2, "OV start eq, update sub-pattern %d\n", subp_i);
						*subp = *cb_ov;
						cd->subp_i = subp_i;
						subp_updated = true;
						break;
					}
					lgdebug(2, "Backtrack handling\n");
				}
			}
			assert(subp_i >= 0, "Recorded sub-pattern index");
			assert(subp_updated);
			cd->capture_level[cd->subp_i] = cb->capture_last;

			if (verbosity >= 6)
			{
				printf("AFTER: subp:\n");
				if (cd->subp_ovfl) printf("OVERFLOW\n"); /* shouldn't happen */
				printov(cb->subject, cd->subp, cd->subp_i+1, cd, /*is_pcreov*/false);
			}

			/* Make a dictionary lookup for NAME in capture groups (?<NAME>x)
			 * (x is a constraint for the initial pattern-match comparison done by
			 * PCRE). */
			 // if (cgnum && * cd->is_constant) printf("is_constant\n");

			/* If we have a cgnum structure with a dict, check if the string to be
			 * matched is in the dict or belongs to the given affix class.
			 * A NULL cgnum->dict means this is a regex from the regex file. */

			if (cgnum && cgnum->dict)
			{  /* && !cd->is_constant */
				int numchar = cb_ov->e - cb_ov->s;

				/* Debug: Sanity check. */
				assert(numchar>=0, "numchar=%d", numchar);
				endname = NULL;
				for (openp = &cd->pattern[cb->pattern_position-5]; *openp; openp--)
				{
					if (*openp == '>') endname = openp;
					if (*openp == '(' && openp[1] == '?' && openp[2] == '<' && openp[3] != '=') break;
				}
				if (NULL != openp && *openp == '(' && NULL != endname && strncmp(openp, "(?<", 3) == 0 && endname > openp)
					; /* Everything is OK. */
				else
				{
					assert(0, "Error: Not in a named group!");
				}
				lgdebug(6, "GROUP NAME %.*s, cgnum %d, ptr %p, numchar %d\n",
						  (int)(endname - openp - 3), openp+3, cb->capture_last-1, cgnum, numchar);
				/* End of debug sanity check. */

				lgdebug(2, "Try match '%.*s': ", numchar, cb->subject+cb_ov->s);

#if 0
				if (0 == numchar)
				{
					lgdebug(2, "Zero match denied\n");
					return 1;
				}
#endif

				if (!is_word(cb->subject+cb_ov->s, numchar, cgnum))
				{
						lgdebug(2, "NO MATCH\n");
						return 1;
				}
				lgdebug(6, "MATCH\n");
			}
		}
#if 0
		if (verbosity >= 6)
		{
			printf("DEBUG subp:\n");
			if (cd->subp_ovfl) printf("OVERFLOW\n"); /* shouldn't happen */
			printov(cb->subject, cd->subp, cd->subp_i+1, cd);
		}
#endif

		// cd->is_constant = false;
		return 0; /* continue to match the rest of the regex */
		break;

#if 0
	case CALLBACK_CONSTANT_START:
		// cd->is_constant = true;
		return 0;
		break;

	case CALLBACK_CONSTANT_END:
		// cd->is_constant = false;
		return 0;
		break;
#endif

	case CALLBACK_END:
		cd->alt_counter++;
		printf("Alternative %d:\n", cd->alt_counter);
		/* See the comment for SUBP0END_DEBUG_SIGNATURE. */
		assert(cd->subp[0].e>=0, "subp[0].e is %d!", cd->subp[0].e);
		printov(cb->subject, cd->subp, cd->subp_i+1, cd, /*is_pcreov*/false);

		/* Remove the last sub-pattern, in case it is a null string (no need to
		 * check, it can be removed anyway since if it is not a null string it is
		 * going to be replaced on the next match). Else the next match, which
		 * will be without this null string, we emit it again as the last
		 * sub-pattern component. FIXME: It doesn't always help. */

		if (cd->subp_i > 0)
		{
			cd->capture_level[cd->subp_i] = -3; /* mark as invalid, for debug */
			cd->subp_i--;
		}

		// cd->is_constant = false;
		return 1; /* signify a backtrack in order to find the next alternative */
		break;

	default:
		assert("Callout: Unreached" && 0);
	}

	return 0; /* Really unreached. */

/*
	printf("Callout %d, data test %d\n"
	       "version %d\n"
			 "subject '%s\n"
			 "subject_length %d\n"
			 "start_match %d\n"
			 "current_position %d\n"
			 "capture_top %d\n"
			 "capture_last %d\n"
			 "pattern_position %d\n"
			 "next_item_length %d\n",
			 cb->callout_number, ((callout_data *)cb->callout_data)->test,
			 cb->version, cb->subject, cb->subject_length, cb->start_match,

			 cb->current_position,
			 cb->capture_top,
			 cb->capture_last,

			 cb->pattern_position,
			 cb->next_item_length);
	return 0;
*/

}

/* Was main() of the test program... */
static int regex_split(const char *inpat, int flags, const char *str, Dictionary dict)
{
	const char *p;
	dyn_str *pat;
	int plevel;  /* paren level */
	int cglevel; /* capture group level */
	int nplevel;  /* paren level within named capture group */
	int cgnum;  /* capture group number*/
	int options;
	const char *errptr;
	int erroffset;
	pcre *pcre;
	const char * const prog = "regex_tokenizer_test";
	int rc;
	pcre_extra *extra = NULL;
#define OVCNT 15
	int ovector[OVCNT];
	callout_data callout_data;

#if 0
	const char **wordlist;
#endif
	bool word_compare_flag = true;
#ifdef notdef
	dyn_str *wordalts;
#endif
	const char *group_name = NULL;
	char *word_classname;
	char c0[2] = "\0\0";

	/* FIXME: validate we use PCRE version 2 at least. */

	/* Find the number of capturing groups in the input pattern. */
	cgnum = 0;
	for (p = inpat; '\0' != *p; p++)
	{
		/* Count as capture groups only (string) or (?<name>). Especially, avoid
		 * counting (?<=...) (positive look behind) and (?(condition)...) (the
		 * (condition) part).
		 * FIXME: support () inside [].
		 * FIXME: support \. */
		if ((*p == '(') && (*p != '*') &&
		    ((p[1] != '?') || ((p[2] == '<') && (p[3] != '='))) &&
			 ((p-inpat < 2) || (p[-2] != '(') || (p[-1] != '?')))
		{
			cgnum++;
		}
	}
	if (0 == cgnum)
	{
		printf("%s: pattern must include at least one () group (was: %s)\n", prog, inpat);
		return 9;
	}
#if 0
	if (p[-1] != '$')
	{
		/* FIXME: add $ if needed */
		printf("%s: pattern must end with $ (was: %s)\n", prog, inpat);
		return 9;
	}
#endif

	/* Regex syntax check of the pattern.
	 * FIXME: Add support for "(?J)" */
	options = PCRE_UTF8;
	pcre = pcre_compile(inpat, options, &errptr, &erroffset, NULL);
	if (NULL == pcre)
	{
		printf("%s: pcre_compile: Error in pattern '%s' at offset %d: %s\n",
		       prog, inpat, erroffset, errptr);
		return 2;
	}

	callout_data.wordlist = NULL;
	callout_data.cgnum = NULL;
	if (word_compare_flag)
	{
		int i;
#if 0
		callout_data.wordlist = malloc(sizeof(*callout_data.wordlist)*cgnum);
#endif
		callout_data.cgnum = malloc(sizeof(*callout_data.cgnum)*cgnum);
		//printf("ALLOCATED callout_data.cgnum %ld for %d groups\n", sizeof(*callout_data.wordlist)*cgnum, cgnum);
		for (i = 0; i < cgnum; i++)
		{
#if 0
			callout_data.wordlist[i] = NULL;
#endif
			callout_data.cgnum[i] = NULL;

		}
	}

	/* Build the pattern that finds all possible matches. */
	pat = dyn_str_new();
	plevel = 0;
	cglevel = 0;
	cgnum = -1; /* First capture group (plevel==1) is cgnum==0. */

	/* Convert the input regex to the tokenizer regex.
	 * cglevel counts named capture groups
	 * plevel counts all groups
	 *
	 * FIXME: Add support for:
	 * (?x) - comment mode.
	 * (?i) - ignore case.
	 * \ - backslash for ()<>?* .
	 * [] - () inside it
	 * FIXME: Add "(?: ... )" over the result pattern.
	 */
	//dyn_strcat(pat, "(?J)");
	for (p = inpat; '\0' != *p; p++)
	{
		char *re = NULL; /* a regex from the 4.0.regex file */

		switch (*p)
		{
		const char *c;

		case '(':
			if (cglevel > 0)
			{
				printf("Error at position %ld: Tokenizer capture groups cannot have nested groups\n", p-inpat);
			}
			plevel++;
			if ((p[1] == '*') ||
			    ((p[1] == '?') && ((p[2] != '<') || (p[3] == '='))) ||
			    ((p-inpat > 1) && (p[-2] == '(') && (p[-1] == '?')))
			{
				break;
			}
			cglevel++;
			if (cglevel > 1)
			{
				printf("Error at position %ld: Tokenizer aregex cannot have capture group level > 1\n", p-inpat);
				free(callout_data.cgnum);
				return 199;
			}
			cgnum++;
			dyn_strcat(pat, "(?:");
			group_name = NULL;
			break;
		case ')':
			plevel--;
			if (cglevel > 0)
			{
				cglevel--;
				/* Add the dict lookup and capturing callback. */
				dyn_strcat(pat, ")(?C)");
			}
			group_name = NULL;
			break;
		case '<':
			/* Remember it as a potential start of a named group. */
			if ((p-2 >= inpat) && (p[-2] == '(') && (p[-1] == '?') && (p[1]  != '='))
			{
				group_name = p + 1;
			}
			else
				group_name = NULL;
			break;
		case '>':
			if (NULL != group_name)
			{
				/* Check if this is actually a group name */
				for (c = group_name; c < p; c++)
				{
					/* FIXME: 'a' and 'p' are part of a hack for lookup_mark.
					 * FIXME: 'r' is part of a hack for regex names that match affix
					 * class names. The fix is not to use matching names. */
					if ((*c > 'Z' || *c < 'A') && *c != 'a' && *c != 'p' && *c != 'r') break;
				}
				if (c == p)
				{
					word_classname = malloc(p-group_name+1);
					strncpy(word_classname, group_name, p-group_name);
					word_classname[p-group_name] = '\0';
				} else
				{
					printf("%s: Invalid class name in group name found at '%s'\n",
					       prog, group_name-4);
					word_classname = NULL;
				}
			} else
			{
					word_classname = NULL;
			}
			if (!word_classname)
			{
				group_name = NULL;
				break;
			}
			dyn_strcat(pat, ">");

			lgdebug(6, "Found word-class %s\n", word_classname);
#if 0
			wordlist = readwords(word_classname);
			if (NULL == wordlist)
			{
				printf("i%s: Invalid class name %s in group name\n", prog, word_classname);
				return 100;
			}

			if (!word_compare_flag)
			{
				printf("Invocation without -w is not supported\n");
				return 103;
			}
#endif

			if (word_compare_flag)
			{
				char *t;
				const char *lookup_mark = NULL;
#if 0
				callout_data.wordlist[cgnum] = wordlist;
				printf("WORDLIST %p at cgnum %d\n", wordlist, cgnum);
#endif
				/* Allocate per group info  */
				callout_data.cgnum[cgnum] = malloc(sizeof(*(callout_data.cgnum)[0]));
				callout_data.cgnum[cgnum]->name = NULL;
				//printf("ALLOCATED cgnum[%d]=%p\n", cgnum, callout_data.cgnum[cgnum]);

				/* A hack for testing: Handle WORDpX or WORDaX.
				 * The above a/p marks mean append/prepend X to word before making
				 * the lookup.
				 * FIXME: Find another way to specify that, maybe in the affix file
				 * or in a tokenizer definition file. */
				t = strpbrk(word_classname, "pa");
				if (NULL != t)
				{
					Afdict_class *ac;

					callout_data.cgnum[cgnum]->lookup_mark_pos = *t;
					*t = '\0';
					ac = afdict_find(dict->affix_table, t+1, /*notify_err*/false);
					if (NULL == ac)
					{
						printf("%s: Unknown afclass '%s'\n", prog, t+1);
						return 253;
					}

					/* Check if the requested affix class is defined and is not an
					 * empty string (like the default INFIXMARK). */
					if (0 == ac->length || '\0' == ac->string[0][0])
					{
						printf("%s: No value for afclass '%s'\n", prog, t+1);
						return 252;
					}
					lookup_mark = ac->string[0]; /* FIXME: support more than one value. */
				}

				callout_data.cgnum[cgnum]->lookup_mark = lookup_mark;
				callout_data.cgnum[cgnum]->name = word_classname;

				if (0 == strcmp(word_classname, "DICTWORD"))
				{
					/* Assign data for looking up a word in the main dict. */
					callout_data.cgnum[cgnum]->dict = dict;
					callout_data.cgnum[cgnum]->afclass = NULL;
				}
				else
				if (afdict_find(dict->affix_table, word_classname, /*notify_err*/false))
				{
					callout_data.cgnum[cgnum]->dict = dict->affix_table;
					callout_data.cgnum[cgnum]->afclass = word_classname;
				}
				else
				{
					if ('r' == word_classname[0]) word_classname++;
					re = get_regex_by_name(dict, word_classname);
					if (re)
					{
						lgdebug(6, "Regex %s with modified groups: '%s'\n", word_classname, re);
						callout_data.cgnum[cgnum]->dict = NULL;
						/* FIXME: No need to allocate callout_data.cgnum[cgnum] in this
						 * case. */
					}
					else
					{
						printf("%s: Unknown word classname '%s'\n", prog, word_classname);
						return 254;
					}
				}
				/* TODO: Assign flags, e.g. for emitting the words with stem/infix marks. */

			} else
			{
#if 0
				wordalts = make_wordalts(wordlist);
				dyn_strcat(pat, wordalts->str);
				dyn_str_delete(wordalts);
				free(wordlist);
#else
				printf("%s: Invocation without -w is not supported\n", prog);
				return 103;
#endif
			}
			/* Default match for dictionary lookup is ".*".
			 * Allow replacing it by something else.
			 * E.g: .{2,}|a */
			if (')' == p[1])
			{
				if (NULL == re)
				{
					dyn_strcat(pat, ".*");
				}
				else
				{
					dyn_strcat(pat, re);
					free(re);
					re = NULL;
				}
			}
			else
			{
				nplevel = 1;
				/* FIXME: Add support for:
				 * (?x) - comment mode.
				 * \ - backslash for ()<>?* .
				 * [] - () inside it
				 */
				for (; p[1] != '\0' && nplevel > 0; p++)
				{
					switch (p[1])
					{
					case '(':
						if (('?' != p[2]) && ('*' != p[2]) &&
						    ((p[-1] != '(') || (p[0] != '?')))
						{
							printf("%s: Capture_group %d: Nested capture group is not supported\n",
							       prog, cgnum+1);
							return 250;
						}
						nplevel++;
						break;
					case ')':
						nplevel--;
						if (0 == nplevel) continue; /* we are done */
						break;
					}

					c0[0] = p[1];
					dyn_strcat(pat, c0);
				}
				p--;
			}

			word_classname = NULL;
			group_name = NULL;
			continue;
		}

		c0[0] = *p;
		dyn_strcat(pat, c0);
	}

	/* Add '$' at the end if needed. */
	if ('$' != pat->str[pat->end-1]) dyn_strcat(pat, "$");
	/* Add the backtracking callback. */
	dyn_strcat(pat, "(?C1)");

	printf("Modified pattern: %s", pat->str);
	lgdebug(2, " (len %zu/%zu)", pat->end, pat->len);
	printf("\n");

	pcre_callout = callout;

	callout_data.function = 1;
	callout_data.subp_i = 0;
	callout_data.subp[0].s = 0;
	callout_data.subp[0].e = SUBP0END_DEBUG_SIGNATURE;
	callout_data.subp_ovfl = false;
	callout_data.capture_last = 0;
	callout_data.pattern = pat->str;
	callout_data.alt_counter = 0;

	options = PCRE_UTF8;
	pcre = pcre_compile(pat->str, options, &errptr, &erroffset, NULL);
	if (NULL == pcre)
	{
		printf("%s: Internal error: pcre_compile: Error in pattern '%s' at offset %d: %s\n",
		       prog, pat->str, erroffset, errptr);
		return 99;
	}

	/* TODO: Check if using JIT may optimize out some needed callouts. */
	options = 0; //PCRE_STUDY_JIT_COMPILE;
	extra  = pcre_study(pcre, options, &errptr);
	if (NULL == extra)
	{
		if (NULL != errptr)
		{
			printf("%s: pcre_study: Error for pattern '%s': %s\n", prog, pat->str, errptr);
			return 3;
		}
		extra = malloc(sizeof(*extra));
		memset(extra, 0, sizeof(*extra));
	} else
	{
		/* For some reason JIT is sometimes done even though it was not requested.
		 * But the callouts are still invoked as expected in such cases. */
		lgdebug(6, "%s: pcre_study: JIT %ld\n", prog, extra->flags & PCRE_STUDY_JIT_COMPILE);
	}

#if 0
	extra->match_limit = 10000;
	extra->match_limit_recursion = 10000;
	extra->flags |= PCRE_EXTRA_MATCH_LIMIT|PCRE_EXTRA_MATCH_LIMIT_RECURSION;
#endif

	extra->callout_data = (void *)&callout_data;
	extra->flags |= PCRE_EXTRA_CALLOUT_DATA;

#if 0
	printf("CGNUM %d\n", cgnum);
	if (NULL != callout_data.cgnum)
	{
		int i;

		for (i = 0; i <= cgnum; i++)
		{
			printf("callout_data.cgnum[%d] %p\n", i, callout_data.cgnum[i]);
		}
	} else
		printf("CGNUM %p\n", callout_data.cgnum);
#endif

	options = PCRE_ANCHORED; /* XXX Maybe PCRE_NO_START_OPTIMIZE is needed too */
	rc = pcre_exec(pcre, extra, str, strlen(str), 0, options, ovector, OVCNT);
	if (rc < 0)
	{
		if (PCRE_ERROR_NOMATCH == rc)
		{
			lgdebug(2, "No match (must always happen)\n");
		} else
		{
			printf("%s: pcre_exec: Error %d\n", prog, rc);
		}
	} else
	{
		printf("Internal error: Unexpected match, rc=%d\n", rc);
	}

	if (0 == rc)
	{
	  rc = OVCNT/3;
	  printf("ovector only has room for %d captured substrings\n", rc - 1);
	}

	printov(str, (ov *)ovector, rc, NULL, /*is_pcreov*/true);

	if (verbosity > 6)
	{
		if (0 != callout_data.subp_i)
		{
			printf("Callout stack:\n");
			printov(str, callout_data.subp, callout_data.subp_i, &callout_data, /*is_pcreov*/false);
		}
	}

	/* Free everything. */
	dyn_str_delete(pat); /* note - callback_data uses parts of pat */
	pcre_free_study(extra); /* safe even if malloc'ed */
	free(pcre);

	if (NULL != callout_data.cgnum)
	{
		int i;

		for (i = 0; i <= cgnum; i++)
		{
			if (callout_data.cgnum[i])
			{
				/* FIXME: Free also word_classname. */
				free(callout_data.cgnum[i]);
			}
		}
		free(callout_data.cgnum);
	}

#if 0
	if (NULL != callout_data.wordlist)
	{
		int i;

		for (i = 0; i < cgnum; i++)
		{
			free(callout_data.wordlist[i]);
		}
		free(callout_data.wordlist);
	}
#endif

	return 0;
}

/**
 * Test the RegEx tokenizer.
 * line - REGEX/,token
 */
int regex_tokenizer_test(Dictionary dict, const char *line)
{
	int linelen = strlen(line);
	char *regex = alloca(linelen+1);
	char *token = alloca(linelen);
	char *regex_end;
	int tokenizer_flags;

	strcpy(regex, line);
	regex_end = index(regex, '/');
	if (NULL == regex_end)
	{
		printf("Missing terminating '/' in regex.\nUsage: /REGEX/,token\n");
		return 101;
	}
	*regex_end = '\0';
	regex_end++;
	/* FIXME: Add iterations for more flags if needed. */
	switch (*regex_end)
	{
		case 'M':
			tokenizer_flags = MARK_TOKENS;
			regex_end++;
			break;
	}
	if (',' != *regex_end)
	{
		printf("Missing terminating ',' after regex end.\nUsage: /REGEX/,token\n");
		return 102;
	}
	strcpy(token, regex_end + 1);
	if ('\0' == token[0])
	{
		printf("Missing token\nUsage: /REGEX/,token\n");
		return 103;
	}

	return regex_split(regex, tokenizer_flags, token, dict);
}

#else /* USE_REGEX_TOKENIZER */

/* Mac OSX will fail to link if this dummy is not defined.
 * But why is it needed?  Because it shows up in the exported
 * symbols list (link-grammar.def) and if its there, it must
 * also be in the code. Thus, the below.
 */
#include "regex-tokenizer.h"
int regex_tokenizer_test(Dictionary dict, const char *line)
{
	return 0;
}
#endif /* USE_REGEX_TOKENIZER */

