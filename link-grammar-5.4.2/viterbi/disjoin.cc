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

#include "compile.h"
#include "disjoin.h"

namespace link_grammar {
namespace viterbi {

/**
 * Order connectors so that all left-pointers appear before all right-
 * pointers.  This is required, as the connect algo always tries to make
 * connections sequentially, so of right-going connectors appear before
 * the left-goers, the left-goers fail to connect.
 *
 * The input to this is presumed to be in DNF.
 */
static Atom* normal_order(Atom* dnf)
{
	
	// Simply recurse on down.
	Or* ora = dynamic_cast<Or*>(dnf);
	if (ora)
	{
		OutList norm;
		foreach_outgoing(Atom*, a, ora)
		{
			norm.push_back(normal_order(a));
		}
		return new Or(norm, ora->_tv);
	}

	And* andy = dynamic_cast<And*>(dnf);
	if (andy)
	{
		OutList norm;
		foreach_outgoing(Connector*, c, andy)
		{
			assert(c, "normal_order: expecting a connector in the disjunct");
			if ('-' == c->get_direction())
				norm.push_back(normal_order(c));
		}
		foreach_outgoing(Connector*, cc, andy)
		{
			assert(cc, "normal_order: expecting a connector in the disjunct");
			if ('+' == cc->get_direction())
				norm.push_back(normal_order(cc));

			// Optional connectors are not allowed to appear in disjuncts!
			assert(not cc->is_optional(), "Optional connector in disjunct");
		}
		return new And(norm, andy->_tv);
	}

	// no-op
	return dnf;
}



/**
 * Convert mixed connector expressions into disjunctive normal form.
 * The final form will consist of disjunction of conjunctions of
 * connectors.
 *
 * Note that connector expressions resemble Boolean logic terms, but
 * they are not actually Boolean logic; each connector can be used
 * once, and only once.  Thus, connector expressions are OR-distributive,
 * but not AND-distributive.  Thus, (A & (B or C)) = ((A & B) or (A & C))
 * but it is not at all the case that (A or (B & C)) is the same as
 * ((A or B) & (A or C)) because connectors cannot be duplicated!
 * That is, the logic of link-grammar connectors is that of linear
 * logic (monoidal categories) not boolean logic (cartesian categories).
 *
 * The primary user of this function is the parser, to convert the 
 * mixed-form dictionary entries into a simpler structure, thus
 * simplifying the parser algorithm.
 *
 * XXX This code somewhat duplicates the function offered by the
 * disjoin() methods on the OR and AND nodes.  BTW, those functions
 * are unit-tested; this one is not.  However, this one handles
 * optionals, the other does not.
 */

Atom* disjoin(Atom* mixed_form)
{
	AtomType intype = mixed_form->get_type();
	if ((OR != intype) and (AND != intype))
		return mixed_form;

	if (OR == intype)
	{
		Or* junct = dynamic_cast<Or*>(mixed_form);
		assert(junct, "disjoin: given a naked OR link!");

		junct = junct->flatten();

		// Just a recursive call, that's all.
		OutList new_oset;
		foreach_outgoing(Atom*, norm, junct)
			new_oset.push_back(disjoin(norm));

		Or* new_or = new Or(new_oset, junct->_tv);
		return normal_order(new_or->super_flatten());
	}

	And* junct = dynamic_cast<And*>(mixed_form);
	assert(junct, "disjoin: mixed form is not AND link!");

	junct = junct->flatten();
	Atom* ajunct = junct->clean();

	// After cleaning, it might be just a single optional clause.
	// e.g. after (A+ or [[()]]) & B+;
	junct = dynamic_cast<And*>(ajunct);
	if (not junct)
		return ajunct;

	// If we are here, the outgoing set is a conjunction of atoms.
	// Search for the first disjunction in that set, and distribute
	// over it.
	OutList front;
	size_t sz = junct->get_arity();
	size_t i;
	for (i=0; i<sz; i++)
	{
		Atom* a = disjoin(junct->get_outgoing_atom(i));
		AtomType t = a->get_type();
		if (OR == t)
			break;
		front.push_back(a);
	}

	/* If no disjunctions found, we are done */
	if (i == sz)
		return normal_order(junct);

	Atom *orat = junct->get_outgoing_atom(i);
	i++;

	OutList rest;
	for(; i<sz; i++)
	{
		Atom* norm = disjoin(junct->get_outgoing_atom(i));
		rest.push_back(norm);
	}

	Or* orn = dynamic_cast<Or*>(orat);
	assert(orn, "Bad link type found during disjoin");

	// Distribute over the elements in OR-list
	OutList new_oset;
	sz = orn->get_arity();
	for (i=0; i<sz; i++)
	{
		OutList distrib;

		// Copy the front, without change.
		// We've got to clone the thing, because otherwise propagating
		// the costs upwards becomes an ugly mess.
		size_t jsz = front.size();
		for (size_t j=0; j<jsz; j++)
			distrib.push_back(front[j]->clone());

		// insert one atom.
		distrib.push_back(orn->get_outgoing_atom(i)->clone());

		// Copy the rest.  Again with the clone(). Irritating.
		jsz = rest.size();
		for (size_t j=0; j<jsz; j++)
			distrib.push_back(rest[j]->clone());

		And *andy = new And(distrib);
		new_oset.push_back(andy->clean());
	}

	Or* new_or = new Or(new_oset, orn->_tv);
	Atom* new_a = new_or->super_flatten();
	new_a = disjoin(new_a);

	Set* newset = dynamic_cast<Set*>(new_a);
	if (newset)
		return normal_order(newset->super_flatten());
	return normal_order(new_a);
}


} // namespace viterbi
} // namespace link-grammar

