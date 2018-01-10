/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright (c) 2009-2013 Linas Vepstas                                 */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/
#ifndef _DICT_FILE_UTILITIES_H_
#define _DICT_FILE_UTILITIES_H_

#include <stdbool.h>
#include <stdio.h>

char * join_path(const char * prefix, const char * suffix);

FILE * dictopen(const char *filename, const char *how);
void * object_open(const char *filename,
                   void * (*opencb)(const char *, const void *),
                   const void * user_data);

bool file_exists(const char * dict_name);
char * get_file_contents(const char *filename);

#endif /* _DICT_FILE_UTILITIES_H_ */
