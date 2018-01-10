/*************************************************************************/
/* Copyright (c) 2005  Sampo Pyysalo                                     */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _REGEX_MORPH_H
#define _REGEX_MORPH_H

#include "dict-common.h"

int compile_regexs(Regex_node *, Dictionary);
const char *match_regex(const Regex_node *, const char *);
void free_regexs(Regex_node *);
#endif /* _REGEX_MORPH_H */
