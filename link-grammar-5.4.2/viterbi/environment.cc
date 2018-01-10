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

#include <math.h>
#include <gc/gc.h>
#include "environment.h"
#include "utilities.h"

namespace atombase {

Environment* Environment::top()
{
	static Environment* global_env = new Environment();
	return global_env;
}

Environment::Environment()
{
}

/// Insert an atom into the environment.
/// The environment keeps a pointer to the atom, so that it won't
/// get garbage collected.
void Environment::insert_atom(Atom* a)
{
	std::lock_guard<std::mutex> lck(_mtx);
	_atoms.insert(a);
}

/// Remove an atom from the environment.
/// If there are no other refrences to the atom, it will be
/// garbage-collected.
void Environment::remove_atom(Atom* a)
{
	std::lock_guard<std::mutex> lck(_mtx);
	_atoms.erase(a);
}

/// Create a relation link, and put it into the environment, all in one go.
Relation* Environment::add_relation(const char* name, Atom* arg, Atom* val)
{
	Relation *rel = new Relation(name, arg, val);
	insert_atom(rel);
	return rel;
}

/// Return a set of all of the relations with the given name, and given
/// argument (value from its domain).
Set* Environment::get_relations(const char* name, Atom* arg)
{
	// XXX this should use a pre-computed index, instead of searching ...
	// XXX FIXME
	OutList oset;
	Label* lab = new Label(name);
	std::set<Atom*>::iterator it = _atoms.begin();
	std::set<Atom*>::iterator end = _atoms.end();
	for (; it != end; ++it)
	{
		Atom* a = *it;
		if (a->get_type() != RELATION) continue;
		Link* l = dynamic_cast<Link*>(a);
		if (l->get_arity() != 3) continue;
		if (not l->get_outgoing_atom(1)->operator==(arg)) continue;
		if (not l->get_outgoing_atom(0)->operator==(lab)) continue;
		oset.push_back(l);
	}

	return new Set(oset);
}

/// Return a set of of the "outputs" (codomain) of a relation, given it's
/// name and input argument (value from its domain).
Set* Environment::get_relation_vals(const char* name, Atom* arg)
{
	Set* relset = get_relations(name, arg);
	OutList oset;
	foreach_outgoing(Relation*, rel, relset)
	{
		oset.push_back(rel->get_outgoing_atom(2));
	}

	return new Set(oset);
}

/// Create a relation link, and put it into the environment, all in one go.
/// If there alerady is a relation with the same name and input value,
/// it is replaced by the new output value.  In other words, this
/// defines a 'function' in the mathematical sense: given a function
/// name and a value from its domain, it returns a single, unique value
/// from its codomain.
///
Relation* Environment::set_function(const char* name, Atom* arg, Atom* val)
{
	Relation* old = get_function(name, arg);
	if (old)
		remove_atom(old);

	Relation *func = new Relation(name, arg, val);
	insert_atom(func);
	return func;
}

Relation* Environment::get_function(const char* name, Atom* arg)
{
	Set* relset = get_relations(name, arg);
	size_t arity = relset->get_arity();
	assert(arity < 2, "Function has bad arity!");
	if (0 == arity) return NULL;

	return dynamic_cast<Relation*>(relset->get_outgoing_atom(0));
}

// Get the value of function 'name' at argument 'arg'
Atom* Environment::get_function_value(const char* name, Atom* arg)
{
	Relation* val = get_function(name, arg);
	if (val) return val->get_outgoing_atom(2);
	return NULL;
}

// Set the value of numeric function 'name' at argument 'arg'
Relation* Environment::set_number(const char* name, Atom* arg, double v)
{
	return set_function(name, arg, new Number(v));
}

// Get the value of numeric function 'name' at argument 'arg'
double Environment::get_number(const char* name, Atom* arg)
{
	Atom* a = get_function_value(name, arg);
	if (!a) return nan("");
	Number* n = upcast<Number*>(a);
	if (!n) return nan("");
	return n->get_value();
}

} // namespace atombase
