/*
 * NNAdjust.h
 *
 * Bumps up word-sense dependencies between nouns in a 
 * noun-modifier relationship with each other.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_NN_ADJUST_H
#define OPENCOG_NN_ADJUST_H

#include <string>

#include "Atom.h"
#include "AtomSpace.h"

namespace opencog {

class NNAdjust
{
	private:
		bool adjust_parse(Handle);
		bool adjust_word(Handle);
		bool adjust_relation(const std::string &, Handle, Handle);

		Handle second_word_inst;
		Handle first_word_sense;
		Handle first_sense_link;
		bool sense_of_first_inst(Handle, Handle);
		bool sense_of_second_inst(Handle, Handle);

	public:
		NNAdjust(void);
		~NNAdjust();
		void adjust_sentence(Handle);
};
}

#endif /* OPENCOG_NN_ADJUST_H */
