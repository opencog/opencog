/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2012 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LINKGRAMMAR_BUILD_DISJUNCTS_H
#define _LINKGRAMMAR_BUILD_DISJUNCTS_H

#include "api-types.h"

Disjunct * build_disjuncts_for_exp(Exp*, const char*, double cost_cutoff);

#ifdef DEBUG
void prt_exp(Exp *, int);
void prt_exp_mem(Exp *, int);
#endif /* DEBUG */

#endif /* _LINKGRAMMAR_BUILD_DISJUNCTS_H */
