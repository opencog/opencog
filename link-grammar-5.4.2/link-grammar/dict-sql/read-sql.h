/*
 * read-sql.h
 *
 * Read in dictionary from an SQL DB.
 * Keeping it simple for just right now, and using SQLite.
 *
 * The goal of reading the dictionary from SQL is to enable some
 * other process (machine-learning algo) to dynamically update
 * the dictionary.
 *
 * Copyright (c) 2014 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifndef READ_SQL_H
#define READ_SQL_H

#include "link-includes.h"

#ifdef HAVE_SQLITE
bool check_db(const char *lang);
Dictionary dictionary_create_from_db(const char *lang);
#else

static inline bool check_db(const char *lang) { return false; }
static inline Dictionary dictionary_create_from_db(const char *lang) { return NULL; }
#endif /* HAVE_SQLITE */

#endif /* READ_SQL_H */
