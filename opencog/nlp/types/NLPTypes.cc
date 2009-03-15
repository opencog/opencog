/**
 * NLPTypes.cc
 *
 * Atom Types used during NLP processing.
 *
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#include "atom_types.definitions"
#include "NLPTypes.h"

using namespace opencog;

void NLPTypes::init_atom_types(void)
{
	#include "atom_types.inheritance"
}


/* ======================= END OF FILE ==================== */
