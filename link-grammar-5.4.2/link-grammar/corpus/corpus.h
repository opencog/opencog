/*
 * corpus.h
 *
 * Data for corpus statistics, used to provide a parse ranking
 * to drive the SAT solver, as well as parse ranking with the
 * ordinary solver.
 *
 * Copyright (c) 2008, 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifndef _LINKGRAMMAR_CORPUS_H
#define _LINKGRAMMAR_CORPUS_H

#include "../link-includes.h"

#ifdef USE_CORPUS

#include "api-types.h"

Corpus * lg_corpus_new(void);
void lg_corpus_delete(Corpus *);

void lg_corpus_score(Linkage);
double lg_corpus_disjunct_score(Linkage, WordIdx);
void lg_corpus_linkage_senses(Linkage);

Sense * lg_get_word_sense(Linkage, WordIdx);
Sense * lg_sense_next(Sense *);
int lg_sense_get_index(Sense *);
const char * lg_sense_get_subscripted_word(Sense *);
const char * lg_sense_get_disjunct(Sense *);
const char * lg_sense_get_sense(Sense *);
double lg_sense_get_score(Sense *);
void lg_sense_delete(Linkage);

#else /* USE_CORPUS */

static inline void lg_corpus_score(Linkage l) {}
static inline void lg_corpus_linkage_senses(Linkage l) {}
static inline void * lg_get_word_sense(Linkage lkg, WordIdx word) { return NULL; }
static inline void * lg_sense_next(void *s) {return NULL; }
static inline const char * lg_sense_get_sense(void *s) { return NULL; }
static inline double lg_sense_get_score(void *s) { return 0.0; }
static inline double lg_corpus_disjunct_score(Linkage linkage, WordIdx w) { return 998.0; }
#endif /* USE_CORPUS */

#endif /* _LINKGRAMMAR_CORPUS_H */
