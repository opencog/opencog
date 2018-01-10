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

#ifndef _ATOMBASE_ENVIRONMENT_H
#define _ATOMBASE_ENVIRONMENT_H

#include "compile-base.h"

namespace atombase {

///  Kind-of-like the opencog AtomSpace ... but smaller, simpler.
class Environment : public gc
{
	public:
		Environment();

		void insert_atom(Atom*);
		void remove_atom(Atom*);

		Relation* add_relation(const char*, Atom*, Atom*);
		Set* get_relations(const char*, Atom*);
		Set* get_relation_vals(const char*, Atom*);

		Relation* set_function(const char*, Atom*, Atom*);
		Relation* get_function(const char*, Atom*);
		Atom* get_function_value(const char*, Atom*);

		Relation* set_number(const char*, Atom*, double);
		double get_number(const char*, Atom*);

		static Environment* top();
	protected:
		// Distinct mutex per environment; this should avoid contention.
		std::mutex _mtx;

		// Set of all atoms in the environment
		std::set<Atom*, std::less<Atom*>, gc_allocator<Atom*> > _atoms;
};

} // namespace atombase

#endif // _ATOMBASE_ENVIRONMENT_H
