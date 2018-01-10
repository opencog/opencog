#ifndef __UTIL_HPP__
#define __UTIL_HPP__

extern "C"
{
#include "link-includes.h"
#include "dict-common/dict-structures.h" // For definition of Exp
}

bool isEndingInterpunction(const char* str);
const char* word(Sentence sent, int w);
void free_linkage_connectors_and_disjuncts(Linkage);
void sat_free_linkages(Sentence, LinkageIdx);
void sat_free_linkages(Sentence);
Exp* null_exp();
void add_anded_exp(Exp*&, Exp*);

#endif
