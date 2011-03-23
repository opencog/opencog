/*
 * opencog/atomspace/PredicateIndex.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Welter Silva <welter@vettalabs.com>
 *
 * Copyright (C) 2008 Linas Vepstas <linasvepstas@gmail.com>
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

#include <opencog/atomspace/FixedIntegerIndex.h>
#include <opencog/atomspace/PredicateEvaluator.h>
#include <opencog/atomspace/VersionHandle.h>

namespace opencog
{
class HandleEntry;

/**
 * Implements an index with additional routines needed for managing 
 * predicates.
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
	public:
		PredicateIndex(void);
		void insertAtom(const Atom*);
		void removeAtom(const Atom*);
		HandleEntry * getHandleSet(int) const;

		void addPredicateIndex(Handle, PredicateEvaluator*)
			throw (InvalidParamException);

		PredicateEvaluator* getPredicateEvaluator(Handle gpnHandle) const;
		HandleEntry* findHandlesByGPN(Handle,
			VersionHandle = NULL_VERSION_HANDLE) const;
};

} //namespace opencog

#endif // _OPENCOG_PREDICATEINDEX_H
