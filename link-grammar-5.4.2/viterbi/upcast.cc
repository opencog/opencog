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

#include <stdlib.h>

#include "compile.h"
#include "compile-base.h"
#include "utilities.h"  // needed for assert

using namespace link_grammar::viterbi;

namespace atombase {

// ============================================================

Atom* Atom::upcaster()
{
	if (!this) return this;

	const Node* n = dynamic_cast<const Node*>(this);
	const Link* l = dynamic_cast<const Link*>(this);

	switch (get_type())
	{
		// Links
		case AND:
			if (dynamic_cast<And*>(this)) return this;
			return new And(l->get_outgoing_set(), _tv);
		case OR:
			if (dynamic_cast<Or*>(this)) return this;
			return new Or(l->get_outgoing_set(), _tv);
		case SEQ:
			if (dynamic_cast<Seq*>(this)) return this;
			return new Seq(l->get_outgoing_set(), _tv);
		case SET:
			if (dynamic_cast<Set*>(this)) return this;
			return new Set(l->get_outgoing_set(), _tv);

		// Nodes
		case NUMBER:
			if (dynamic_cast<Number*>(this)) return this;
			return new Number(atof(n->get_name().c_str()), _tv);

		case CONNECTOR:
			if (dynamic_cast<Connector*>(this)) return this;
			return new Connector(n->get_name(), _tv);

		case WORD:
			if (dynamic_cast<Word*>(this)) return this;
			return new Word(n->get_name(), _tv);

		default:
			assert(0, "Atom::upcaster(): implement me!");
	}
}

// ============================================================

Link* Link::append(Atom* a) const
{
	OutList ol = get_outgoing_set();
	ol.push_back(a);

	switch (get_type())
	{
		// Links
		case AND:
			return new And(ol, _tv);
		case OR:
			return new Or(ol, _tv);
		case SEQ:
			return new Seq(ol, _tv);
		case SET:
			return new Set(ol, _tv);

		default:
			std::cerr << "unhandled atom type "<< type_name(get_type()) << std::endl;
			assert(0, "atombase::Link::append: implement me!");
	}
}

// ============================================================

/// Replace stary with nowy.
Link* Link::replace(Atom* novi, Atom* ctari) const
{
	OutList ol = get_outgoing_set();

	// Loo for stary, replace with nowy if found.
	size_t sz = ol.size();
	size_t i = 0;
	for (; i<sz; i++)
	{
		if (ctari == ol[i])
		{
			ol[i] = novi;
			break;
		}
	}
	if (i == sz)
		ol.push_back(novi);

	switch (get_type())
	{
		// Links
		case AND:
			return new And(ol, _tv);
		case OR:
			return new Or(ol, _tv);
		case SEQ:
			return new Seq(ol, _tv);
		case SET:
			return new Set(ol, _tv);

		default:
			std::cerr << "unhandled atom type "<< type_name(get_type()) << std::endl;
			assert(0, "atombase::Link::replace: implement me!");
	}
}

} // namespace atombase
