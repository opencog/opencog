/*
 * NNAdjust.h
 *
 * Bumps up word-sense dependencies between nouns in a 
 * noun-modifier relationship with each other.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_NN_ADJUST_H
#define _OPENCOG_NN_ADJUST_H

#include <string>

#include <opencog/atomspace/Atom.h>

namespace opencog {

class NNAdjust
{
	private:
		double strength_adjust;

		bool adjust_parse_f(Handle);
		bool adjust_word(Handle);
		bool adjust_relation(const std::string &, Handle, Handle);

		Handle second_word_inst;
		Handle first_sense_link;
		bool sense_of_first_inst(Handle, Handle);
		bool sense_of_second_inst(Handle, Handle);
		bool sense_pair(Handle);

	public:
		NNAdjust(void);
		~NNAdjust();
		void adjust_sentence(Handle);
		void adjust_parse(Handle);
};

} // namespace opencog

#endif // _OPENCOG_NN_ADJUST_H
