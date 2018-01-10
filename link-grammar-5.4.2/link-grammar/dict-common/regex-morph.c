/*************************************************************************/
/* Copyright (c) 2005  Sampo Pyysalo                                     */
/* Copyright (c) 2009 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

/* On MS Windows, regex.h fails to pull in size_t, so work around this by
 * including <stddef.h> before <regex.h> (<sys/types.h> is not enough) */
#include <stddef.h>
#include <regex.h>
#include "api-structures.h"
#include "error.h"          /* verbosity */
#include "externs.h"        /* lgdebug() */
#include "dict-api.h"
#include "dict-common.h"
#include "link-includes.h"
#include "regex-morph.h"


/**
 * Support for the regular-expression based token matching system
 * using standard POSIX regex.
 */

/**
 * Notify an error message according to the error code.
 */
static void prt_regerror(const char *msg, const Regex_node *re, int rc)
{
	const size_t errbuf_size = regerror(rc, re->re, NULL, 0);
	char * const errbuf = malloc(errbuf_size);

	/*
	prt_error("Error: Failed to compile regex '%s' (%s) at %d: %s\n",
					re->pattern, re->name, erroroffset, error);
	*/
	regerror(rc, re->re, errbuf, errbuf_size);
	prt_error("Error: %s: \"%s\" (%s): %s\n", msg, re->pattern, re->name, errbuf);
	free(errbuf);
}

/**
 * Compiles all the given regexs. Returns 0 on success,
 * else an error code.
 */
int compile_regexs(Regex_node *re, Dictionary dict)
{
	regex_t *preg;
	int rc;

	while (re != NULL)
	{
		/* If re->re non-null, assume compiled already. */
		if(re->re == NULL)
		{
			/* Compile with default options (0) and default character
			 * tables (NULL). */
			/* re->re = pcre_compile(re->pattern, 0, &error, &erroroffset, NULL); */
			preg = (regex_t *) malloc (sizeof(regex_t));
			re->re = preg;

			/* REG_ENHANCED is needed for OS X to support \w etc. */
#ifndef REG_ENHANCED
#define REG_ENHANCED 0
#endif
			rc = regcomp(preg, re->pattern, REG_EXTENDED|REG_ENHANCED);
			if (rc)
			{
				prt_regerror("Failed to compile regex", re, rc);
				return rc;
			}

			/* Check that the regex name is defined in the dictionary. */
			if ((NULL != dict) && !boolean_dictionary_lookup(dict, re->name))
			{
				/* TODO: better error handing. Maybe remove the regex? */
				prt_error("Error: Regex name %s not found in dictionary!\n",
				       re->name);
			}
		}
		re = re->next;
	}
	return 0;
}

/**
 * Tries to match each regex in turn to word s.
 * On match, returns the name of the first matching regex.
 * If no match is found, returns NULL.
 */
#define D_MRE 6
const char *match_regex(const Regex_node *re, const char *s)
{
	int rc;
	const char *nre_name;

	while (re != NULL)
	{
		/* Make sure the regex has been compiled. */
		assert(re->re);

#if 0
		/* Try to match with no extra data (NULL), whole str
		 * (0 to strlen(s)), and default options (second 0). */
		int rc = pcre_exec(re->re, NULL, s, strlen(s), 0,
		                   0, ovector, PCRE_OVEC_SIZE);
#endif

		rc = regexec((regex_t*) re->re, s, 0, NULL, 0);
		if (0 == rc)
		{
			lgdebug(+D_MRE, "%s%s %s\n", &"!"[!re->neg], re->name, s);
			if (!re->neg)
				return re->name; /* Match found - return--no multiple matches. */

			/* Negative match - skip this regex name. */
			for (nre_name = re->name; re->next != NULL; re = re->next)
			{
				if (strcmp(nre_name, re->next->name) != 0) break;
			}
		}
		else if (rc != REG_NOMATCH)
		{
			/* We have an error. */
			prt_regerror("Regex matching error", re, rc);
		}
		re = re->next;
	}
	return NULL; /* No matches. */
}
#undef D_MRE

/**
 * Delete associated storage
 */
void free_regexs(Regex_node *re)
{
	while (re != NULL)
	{
		Regex_node *next = re->next;

		/* Prevent a crash in regfree() in case of a regex compilation error. */
		if (NULL != re->re)
			regfree((regex_t *)re->re);

		free(re->re);
		free(re->name);
		free(re->pattern);
		free(re);
		re = next;
	}
}
