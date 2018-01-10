/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2014 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include "dict-common/dict-api.h"

/* The structure below stores a list of dictionary word files. */
struct Word_file_struct
{
	Word_file * next;
	const char *file;    /* the file name */
	bool changed;        /* TRUE if this file has been changed (XXX unused) */
};

void free_Word_file(Word_file * wf);

Dict_node * read_word_file(Dictionary dict, Dict_node * dn, char * filename);

void patch_subscript(char *);

