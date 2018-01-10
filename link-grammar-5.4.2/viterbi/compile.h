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

#ifndef _LG_VITERBI_COMPILELG_H
#define _LG_VITERBI_COMPILELG_H

#include "utilities.h"  // needed for assert

#include "atom.h"
#include "compile-base.h"

namespace link_grammar {
namespace viterbi {

using namespace atombase;

#define OPTIONAL_CLAUSE "0"

// Classes that convert run-time atom types into compile-time static
// types, so that the compiler can check these for correctness.
// These are here purely for C++ programming convenience; the true
// structure that matters is the dynamic run-time (hyper-)graphs.

class Connector : public Node
{
	public:
		// Last letter of the connector must be + or -
		// indicating the direction of the connector.
		Connector(const NameString& name, const TV& tv = TV())
			: Node(CONNECTOR, name, tv)
		{
			if (name == OPTIONAL_CLAUSE)
				return;
			char dir = *name.rbegin();
			assert (('+' == dir) or ('-' == dir), "Bad direction");
		}

		bool is_optional() const
		{
			return _name == OPTIONAL_CLAUSE;
		}

		char get_direction() const
		{
			return *_name.rbegin();
		}
      virtual Connector* clone() const { return new Connector(*this); }
};

class LingType : public Node
{
	public:
		LingType(const NameString& name, const TV& tv = TV())
			: Node(LING_TYPE, name, tv)
		{}
};

class Word : public Node
{
	public:
		Word(const NameString& name, const TV& tv = TV())
			: Node(WORD, name, tv)
		{}
};

#if 0
// Atom types.  Right now an enum, but maybe should be dynamic!?
enum AtomType
{
	// Link types
	WORD_DISJ,  // word, followed by a single disjunct for that word.
};
#endif


/// Create a ling-grammar link. This will be of the form:
///     LING:
///        Ling_TYPE "MVa"
///        Atom ...
///        Atom ...
/// where the Atoms are typically either connectors, or WORD_DISJ
///
class Ling : public atombase::Link
{
	public:
		Ling(const OutList& ol)
			: Link(LING, ol)
		{
			assert(3 == ol.size(), "LG link wrong size");
			assert(ol[0]->get_type() == LING_TYPE, "LG link has bad first node");
		}
		Ling(const NameString& str, Atom* a, Atom *b)
			: Link(LING, new LingType(str), a, b) {}

		Ling(LingType* t, Atom* a, Atom *b)
			: Link(LING, t, a, b) {}

		LingType* get_ling_type() const
		{
			return dynamic_cast<LingType*>(get_outgoing_atom(0));
		}

		Atom* get_left() const
		{
			return get_outgoing_atom(1);
		}
		Atom* get_right() const
		{
			return get_outgoing_atom(2);
		}
};


class WordCset : public atombase::Link
{
	public:
		WordCset(Word* a, Atom* b)
			: Link(WORD_CSET, a, b)
		{
			// this should be pointing at:
			// WORD_CSET :
			//   WORD : blah.v
			//   AND :
			//      CONNECTOR : Wd-  etc...

			assert(a->get_type() == WORD, "CSET is expecting WORD as first arg");
			bool ok = false;
			ok = ok or b->get_type() == CONNECTOR;
			ok = ok or b->get_type() == AND;
			ok = ok or b->get_type() == OR;
			assert(ok, "CSET is expecting connector set as second arg");
		}

		Word* get_word() const
		{
			return dynamic_cast<Word*>(_oset[0]);
		}
		Atom* get_cset() const
		{
			return _oset[1];
		}
		bool has_left_pointers() const;
		WordCset* flatten();
};

/// A triple of three sequences.  The first sequence is a sequence of 'input'
/// words (a sentence or phrase that has not yet been parsed).  The second
/// sequence is the current parse state.  The third sequence is the 'output'
/// of the parse, i.e. a set of connected words.
///
/// It of the form
///
///    STATE_TRIPLE :
///       SEQ
///           WORD
///           WORD
///       SEQ
///           WORD_CSET ...
///           WORD_CSET ...
///       SET
///           LING ...
///           LING ...
///
class StateTriple : public atombase::Link
{
	public:
		StateTriple(Seq* input, Seq* state, Set* output)
			: Link(STATE_TRIPLE, input, state, output) {}
		Seq* get_input() const { return dynamic_cast<Seq*>(_oset.at(0)); }
		Seq* get_state() const { return dynamic_cast<Seq*>(_oset.at(1)); }
		Set* get_output() const { return dynamic_cast<Set*>(_oset.at(2)); }
};


} // namespace viterbi
} // namespace link-grammar

#endif // _LG_VITERBI_COMPILELG_H
