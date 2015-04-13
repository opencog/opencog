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

// link-grammar include files, needed for Exp, Dict
#include <link-grammar/dict-api.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/nlp/lg-dict/LGDictReader.h>

#ifdef LATER
#include "compile.h"
#endif

namespace viterbi {

using namespace opencog;

class Parser
{
	public:
		Parser(Dictionary dict, AtomSpace* as);
		~Parser();

#if 0
		void streamin(const std::string&);
		void stream_word_conset(WordCset*);
#endif
		void stream_word(const std::string&);

		Handle word_consets(const std::string& word);

#if 0
		Set* get_alternatives();
#endif
	protected:
		static const char *alternatives_anchor;
		void initialize_state();

		Dictionary _dict;
		SchemeEval* _scm_eval;
		LGDictReader* _dict_reader;
	private:
};

} // namespace viterbi

#endif // _LG_VITERBI_PARSER_H
