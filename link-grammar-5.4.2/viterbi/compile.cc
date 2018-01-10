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

#include "compile.h"

// XXX temporary hack until dynamic types are supported !?
namespace atombase {
using namespace link_grammar::viterbi;

// ============================================================
/// Remove optional connectors.
///
/// It doesn't make any sense at all to have an optional connector
/// in an AND-clause, so just remove it.  (Well, OK, it "makes sense",
/// its just effectively a no-op, and so doesn't have any effect.  So,
/// removing it here simplifies logic in other places.)
///
/// The cost of the optional is passed up to the dsjunct. The reason for
/// this is that the doctionary contains entries such as
/// <post-nominal-x>, whcih has (Xc+ or <costly-null>) & MX-
/// After being disjoined, we need to pass that cost up.
Atom* And::clean() const
{
	TV tv = _tv;
	OutList cl;
	size_t sz = get_arity();

	// Special case: it could be a and-clause containing a single,
	// optional connector, in which case, we flatten the thing
	// (returning the optional connector!)
	if (1 == sz)
	{
		Atom* a = get_outgoing_atom(0);
		a->_tv += _tv;
		return a;
	}
	for (size_t i=0; i<sz; i++)
	{
		Connector* cn = dynamic_cast<Connector*>(get_outgoing_atom(i));
		if (cn and cn->is_optional())
		{
			tv += cn->_tv;
			continue;
		}

		cl.push_back(_oset[i]);
	}

	return new And(cl, tv);
}

} // namespace atombase

// ============================================================

namespace link_grammar {
namespace viterbi {


// Helper function for below.
static bool has_lefties(Atom* a)
{
	Connector* c = dynamic_cast<Connector*>(a);
	if (c)
	{
		if ('-' == c->get_direction())
			return true;
		return false;
	}

	// Verify we've got a valid disjunct
	AtomType at = a->get_type();
	assert ((at == OR) or (at == AND), "Disjunct, expecting OR or AND");

	// Recurse down into disjunct
	Link* l = dynamic_cast<Link*>(a);
	size_t sz = l->get_arity();
	for (size_t i=0; i<sz; i++)
	{
		if (has_lefties(l->get_outgoing_atom(i)))
			return true;
	}
	return false;
}

/// Return true if any of the connectors in the cset point to the left.
bool WordCset::has_left_pointers() const
{
	return has_lefties(get_cset());
}

/// Simplify any gratuituousnesting in the cset.
WordCset* WordCset::flatten()
{
	// AND and OR inherit from Set
	Set* s = dynamic_cast<Set*>(get_cset());
	if (NULL == s)
		return this;

	Atom* flat = s->super_flatten();

	// If there is nothing left after flattening, return NULL.
	const Link* fl = dynamic_cast<const Link*>(flat);
	if (fl && 0 == fl->get_arity())
		return NULL;

	return new WordCset(get_word(), flat);
}


} // namespace viterbi
} // namespace link-grammar
