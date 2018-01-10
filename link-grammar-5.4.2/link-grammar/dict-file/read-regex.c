/*************************************************************************/
/* Copyright (c) 2005 Sampo Pyysalo                                      */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <string.h>
#include "link-includes.h"
#include "api-structures.h"
#include "dict-common/dict-common.h"
#include "dict-common/file-utils.h"
#include "read-regex.h"

/*
  Function for reading regular expression name:pattern combinations
  into the Dictionary from a given file.

  The format of the regex file is as follows:

  Lines starting with "%" are comments and are ignored.
  All other nonempty lines must follow the following format:

      REGEX_NAME:  /pattern/

  here REGEX_NAME is an identifying unique name for the regex.
  This name is used to determine the disjuncts that will be assigned to
  tokens matching the pattern, so in the dictionary file (e.g. 4.0.dict)
  you must have something like

     REGEX_NAME:  (({@MX+} & (JG- or <noun-main-s>)) or YS+)) or AN+ or G+);

  using the same name. The pattern itself must be surrounded by slashes.
  Extra whitespace is ignored.
*/

#define MAX_REGEX_NAME_LENGTH 50
#define MAX_REGEX_LENGTH      255

int read_regex_file(Dictionary dict, const char *file_name)
{
	Regex_node **tail = &dict->regex_root; /* Last Regex_node * in list */
	Regex_node *new_re;
	char name[MAX_REGEX_NAME_LENGTH];
	char regex[MAX_REGEX_LENGTH];
	int c,prev,i,line=1;
	FILE *fp;

	fp = dictopen(file_name, "r");
	if (fp == NULL)
	{
		prt_error("Error: cannot open regex file %s\n", file_name);
		return 1;
	}

	/* read in regexs. loop broken on EOF. */
	while (1)
	{
		bool neg = false;

		/* skip whitespace and comments. */
		do
		{
			do
			{
				c = lg_fgetc(fp);
				if (c == '\n') { line++; }
			}
			while (lg_isspace(c));

			if (c == '%')
			{
				while ((c != EOF) && (c != '\n')) { c = lg_fgetc(fp); }
				line++;
			}
		}
		while (lg_isspace(c));

		if (c == EOF) { break; } /* done. */

		/* read in the name of the regex. */
		i = 0;
		do
		{
			if (i > MAX_REGEX_NAME_LENGTH-1)
			{
				prt_error("Error: Regex name too long on line %d\n", line);
				goto failure;
			}
			name[i++] = c;
			c = lg_fgetc(fp);
		}
		while ((!lg_isspace(c)) && (c != ':') && (c != EOF));
		name[i] = '\0';

		/* Skip possible whitespace after name, expect colon. */
		while (lg_isspace(c))
		{
			if (c == '\n') { line++; }
			c = lg_fgetc(fp);
		}
		if (c != ':')
		{
			prt_error("Error: Regex missing colon on line %d\n", line);
			goto failure;
		}

		/* Skip whitespace after colon, expect slash. */
		do
		{
			if (c == '\n') { line++; }
			c = lg_fgetc(fp);
		}
		while (lg_isspace(c));
		if (c == '!')
		{
			neg = true;
			do
			{
				if (c == '\n') { line++; }
				c = lg_fgetc(fp);
			}
			while (lg_isspace(c));
		}
		if (c != '/')
		{
			prt_error("Error: Regex missing leading slash on line %d\n", line);
			goto failure;
		}

		/* Read in the regex. */
		prev = 0;
		i = 0;
		do
		{
			if (i > MAX_REGEX_LENGTH-1)
			{
				prt_error("Error: Regex too long on line %d\n", line);
				goto failure;
			}
			prev = c;
			c = lg_fgetc(fp);
			regex[i++] = c;
		}
		while ((c != '/' || prev == '\\') && (c != EOF));
		regex[i-1] = '\0';

		/* Expect termination by a slash. */
		if (c != '/')
		{
			prt_error("Error: Regex missing trailing slash on line %d\n", line);
			goto failure;
		}

		/* Create new Regex_node and add to dict list. */
		new_re = (Regex_node *) malloc(sizeof(Regex_node));
		new_re->name    = strdup(name);
		new_re->pattern = strdup(regex);
		new_re->neg     = neg;
		new_re->re      = NULL;
		new_re->next    = NULL;
		*tail = new_re;
		tail = &new_re->next;
	}

	fclose(fp);
	return 0;
failure:
	fclose(fp);
	return 1;
}

