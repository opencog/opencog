/**
 * NLPTypes.cc
 *
 * Atom Types used during NLP processing.
 *
 * Copyright (c) 2009, 2014 Linas Vepstas <linasvepstas@gmail.com>
 */

#include <opencog/server/Module.h>
#include "opencog/nlp/types/atom_types.definitions"

#define INHERITANCE_FILE "opencog/nlp/types/atom_types.inheritance"
#define INITNAME nlp_types_init

#include <opencog/atomspace/atom_types.cc>

using namespace opencog;
TRIVIAL_MODULE(NLPTypesModule)
DECLARE_MODULE(NLPTypesModule)
