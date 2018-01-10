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

#ifndef _LG_READ_DICT_H_
#define  _LG_READ_DICT_H_

#include "dict-common/dict-structures.h"

void print_dictionary_data(Dictionary dict);
void print_dictionary_words(Dictionary dict);

Dictionary dictionary_create_from_file(const char * lang);
bool read_dictionary(Dictionary dict);

Dict_node * file_lookup_list(const Dictionary dict, const char *s);
Dict_node * file_lookup_wild(Dictionary dict, const char *s);
bool file_boolean_lookup(Dictionary dict, const char *s);
void file_free_lookup(Dict_node *llist);

void free_insert_list(Dict_node *ilist);
void insert_list(Dictionary dict, Dict_node * p, int l);

#endif /* _LG_READ_DICT_H_ */
