/*************************************************************************/
/* Copyright (c) 2012, 2013 Linas Vepstas <linasvepstas@gmail.com>       */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _ATOMBASE_ATOM_TYPES_H
#define _ATOMBASE_ATOM_TYPES_H

#include <iostream>

namespace atombase {

// Atom types.  Right now an enum, but maybe should be dynamic!?
enum AtomType
{
	// Generic Node types
	NODE = 1,
	INDEX,
	LABEL,
	NUMBER,

	// Viterbi-specific Node types
	WORD,       // a word
	LING_TYPE,  // a pair of merged connectors (LG LINK TYPE)
	// META,       // special-word, e.g. LEFT-WALL, RIGHT-WALL
	CONNECTOR,  // e.g. S+

	// Generic Link types
	LINK,
	RELATION,   // model-theoretic relation (OpenCog ExecutionLink)
	SET,        // unordered multiset of children
	UNIQ,       // unordered set of children
	SEQ,        // ordered sequence of children
	AND,        // ordered AND of all children (order is important!)
	OR,         // unordered OR of all children

	// Viterbi-specific Link types
	WORD_CSET,  // word, followed by a set of connectors for that word.
	WORD_DISJ,  // word, followed by a single disjunct for that word.
	LING,       // two connected connectors, (LGLINK) e.g. Dmcn w/o direction info
	STATE_TRIPLE, // Current pending input, parse state and corresponding output.

	RULE,       // Base class for graph re-write rules
};

const std::string type_name(AtomType);
std::ostream& operator<<(std::ostream& out, AtomType t);

} // namespace atombase

#endif // _ATOMBASE_ATOM_TYPES_H
