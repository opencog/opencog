/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _TOKENIZE_H
#define _TOKENIZE_H

#include "api-types.h"
#include "link-includes.h"

bool separate_sentence(Sentence, Parse_Options);
bool sentence_in_dictionary(Sentence);
bool flatten_wordgraph(Sentence, Parse_Options);
void tokenization_done(Sentence, Gword *);

void altappend(Sentence, const char ***, const char *);

Gword *issue_word_alternative(Sentence sent, Gword *unsplit_word,
                     const char *label,
                     int prefnum, const char * const *prefix,
                     int stemnum, const char * const *stem,
                     int suffnum, const char * const *suffix);
#endif /* _TOKENIZE_H */
