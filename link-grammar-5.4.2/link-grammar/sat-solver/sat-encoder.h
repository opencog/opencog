#ifndef _SAT_ENCODER_H
#define _SAT_ENCODER_H

#include "link-includes.h"

#ifdef USE_SAT_SOLVER
int sat_parse(Sentence sent, Parse_Options  opts);
Linkage sat_create_linkage(LinkageIdx k, Sentence sent, Parse_Options  opts);
void sat_sentence_delete(Sentence sent);
#else
static inline int sat_parse(Sentence sent, Parse_Options  opts) { return -1; }
static inline Linkage sat_create_linkage(LinkageIdx k, Sentence sent, Parse_Options  opts) { return NULL; }
static inline void sat_sentence_delete(Sentence sent) {}
#endif

#endif /* _SAT_ENCODER_H */
