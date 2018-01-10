/*************************************************************************/
/* Copyright (c) 2013 Linas Vepstas <linasvepstas@gmail.com>             */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LG_VITERBI_REWRITE_H
#define _LG_VITERBI_REWRITE_H

#include "atom.h"

namespace link_grammar {
namespace viterbi {
using namespace atombase;

// Base class for all graph-rewrite rules.
// For now, most/all rewrite rules will be implemented in C++
// Eventually, the goal is not to do this, but for now, this seems
// like the shortest path to something functional.  So assorted
// misc algrothms will inherit from this class.
class Rule : public atombase::Link
{
	public:
		Rule(void)
			: Link(RULE, new Node(""))
		{}
		Node* rule_name() { return dynamic_cast<Node*>(_oset[0]); }
		Atom* apply(Atom* a) { return a; }
};


} // namespace viterbi
} // namespace link-grammar

#endif // _LG_VITERBI_REWRITE_H

