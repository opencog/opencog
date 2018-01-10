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

#ifndef _RESOURCES_H
#define _RESOURCES_H

#include "api-types.h"
#include "link-includes.h"

void      print_time(Parse_Options opts, const char * s);
void      print_total_space(Parse_Options opts);
void      resources_reset(Resources r);
void      resources_reset_space(Resources r);
bool      resources_timer_expired(Resources r);
bool      resources_memory_exhausted(Resources r);
bool      resources_exhausted(Resources r);
Resources resources_create(void);
void      resources_delete(Resources ti);
#endif /* _RESOURCES_H */
