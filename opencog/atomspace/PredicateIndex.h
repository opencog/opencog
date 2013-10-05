/*
 * opencog/atomspace/PredicateIndex.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008,2013 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Welter Silva <welter@vettalabs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_PREDICATEINDEX_H
#define _OPENCOG_PREDICATEINDEX_H

#include <map>
#include <vector>

#include <opencog/util/exceptions.h>
#include <opencog/atomspace/FixedIntegerIndex.h>
#include <opencog/atomspace/PredicateEvaluator.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Implements an index with additional routines needed for managing 
 * predicates.  The general idea is this: first, one inserts a bunch
 * of predicates, using addPredicateIndex(). Each predicate is just a
 * unary function that takes a Handle and returns a bool, true or false. 
 *
 * Next, one inserts a bunch of atoms.  With each atom inserted, every
 * predicate is applied to each atom. If the predicate accepts the atom,
 * then the atom is associated with the corresponding predicate Handle.
 *
 * Later on, by using findHandlesByGPN, one can get back the set of all
 * of the atoms that were associated with that predicate handle (viz,
 * all of the atoms that were accepted by the predicate function).
 */
class PredicateIndex:
	public FixedIntegerIndex
{
	private:
		std::vector<Handle> predicateHandles;
		std::vector<PredicateEvaluator*> predicateEvaluators;
		int numberOfPredicateIndices;
		// Map from each PredicateNode Handle to its corresponding index
		std::map<Handle, int> predicateHandles2Indices;
		const UnorderedHandleSet& getHandleSet(int) const;
	public:
		PredicateIndex(void);
		void insertAtom(AtomPtr);
		void removeAtom(AtomPtr);

		void addPredicateIndex(Handle, PredicateEvaluator*)
			throw (InvalidParamException);

		PredicateEvaluator* getPredicateEvaluator(Handle gpnHandle) const;
		const UnorderedHandleSet& findHandlesByGPN(Handle) const;
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_PREDICATEINDEX_H
