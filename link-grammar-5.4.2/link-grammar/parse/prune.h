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

#ifndef _PRUNE_H
#define _PRUNE_H

#include "link-includes.h"

int        power_prune(Sentence, Parse_Options);
void       pp_and_power_prune(Sentence, Parse_Options);

#endif /* _PRUNE_H */
