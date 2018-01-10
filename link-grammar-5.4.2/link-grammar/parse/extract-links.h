/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this softwares.   */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _EXTRACT_LINKS_H
#define _EXTRACT_LINKS_H

#include "api-structures.h"
#include "link-includes.h"

typedef struct extractor_s extractor_t;

extractor_t* extractor_new(int nwords, unsigned int rand_state);
void free_extractor(extractor_t*);

bool build_parse_set(extractor_t*, Sentence,
                     fast_matcher_t*, count_context_t*,
                     unsigned int null_count, Parse_Options);

void extract_links(extractor_t*, Linkage);

#endif /* _EXTRACT_LINKS_H */
