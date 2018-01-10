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

#ifndef _EXTERNS_H
#define _EXTERNS_H
/* verbosity global is held in api.c */
extern int verbosity;          /* the verbosity level for error messages */
extern char * debug;           /* comma-separated functions/files to debug */
extern char * test;            /* comma-separated features to test */
#endif /* _EXTERNS_H */
