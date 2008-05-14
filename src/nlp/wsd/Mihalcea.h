/*
 * Mihalcea.h
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_MIHALCEA_H
#define OPENCOG_MIHALCEA_H
#include "Atom.h"

namespace opencog {

class Mihalcea
{
	private:

	public:
		Mihalcea(void);
		~Mihalcea();
		void process_sentence(Handle);
};
}

#endif /* OPENCOG_MIHALCEA_H */
