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

#include <opencog/atoms/base/Atom.h>

namespace opencog {

class NNAdjust
{
	private:
		double strength_adjust;

		bool adjust_parse_f(const Handle&);
		bool adjust_word(const Handle&);
		bool adjust_relation(const std::string &, const Handle&, const Handle&);

		Handle second_word_inst;
		Handle first_sense_link;
		bool sense_of_first_inst(const Handle&, const Handle&);
		bool sense_of_second_inst(const Handle&, const Handle&);
		bool sense_pair(const Handle&);

	public:
		NNAdjust(void);
		~NNAdjust();
		void adjust_sentence(const Handle&);
		void adjust_parse(const Handle&);
};

} // namespace opencog

#endif // _OPENCOG_NN_ADJUST_H
