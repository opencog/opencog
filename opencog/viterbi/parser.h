/*************************************************************************/
/* Copyright (c) 2012 Linas Vepstas <linasvepstas@gmail.com>             */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LG_VITERBI_PARSER_H
#define _LG_VITERBI_PARSER_H

#include <string>

#include <opencog/atomspace/Atom.h>

#ifdef LATER
#include "compile.h"
#include "garbage.h"

// link-grammar include files, needed for Exp, Dict
#include "api-types.h"
#include "structures.h"
#endif

namespace viterbi {

using namespace opencog;
struct Exp;

class Parser
{
	public:
		Parser(Dictionary dict);

#if 0
		void streamin(const std::string&);
		void stream_word(const std::string&);
		void stream_word_conset(WordCset*);

		Set* word_consets(const std::string& word);

		Set* get_alternatives();
#endif
	protected:
		void initialize_state();
		Atom* lg_exp_to_atom(Exp*);

		Dictionary _dict;
	private:
		// Set* _alternatives;
};

} // namespace viterbi

#endif // _LG_VITERBI_PARSER_H
