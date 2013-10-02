/*
 * opencog/atomspace/PredicateIndex.cc
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

#include <opencog/atomspace/PredicateIndex.h>
#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/TLB.h>

using namespace opencog;

PredicateIndex::PredicateIndex(void)
{
	resize(MAX_PREDICATE_INDICES);
	predicateHandles.resize(MAX_PREDICATE_INDICES, Handle::UNDEFINED);
	predicateEvaluators.resize(MAX_PREDICATE_INDICES, NULL);
	numberOfPredicateIndices = 0;
}

void PredicateIndex::insertAtom(AtomPtr atom)
{
	// Checks Atom against predicate indices and insert it if needed
	// That is, the atom is inserted only if the predicate says "yes"
	for (int i = 0; i < numberOfPredicateIndices; i++) {
		Handle h = atom->getHandle();
		PredicateEvaluator* evaluator = predicateEvaluators[i];
		if (evaluator->evaluate(h)) {
			// FixIndegerIndex is a set of handles for each index i.
			insert(i, h);
		}
	}
}

void PredicateIndex::removeAtom(AtomPtr atom)
{
	Handle h = atom->getHandle();
	// Erase the handle from each set ... there is a set per index.
	std::vector<UnorderedHandleSet>::iterator s;
	for (s = idx.begin(); s != idx.end(); ++s) {
		s->erase(h);
	}
}

const UnorderedHandleSet& PredicateIndex::getHandleSet(int index) const
{
	return idx.at(index);
}

// ================================================================
/**
 * Adds a new predicate index to this atom table given the Handle of
 * the PredicateNode.
 * @param The handle of the predicate node, whose name is the id
 *        of the predicate.
 * @param The evaluator used to check if such predicate is true
 *        for a given handle.
 * Throws exception if:
 *      - the given Handle is not in the AtomTable
 *      - there is already an index for this predicate id/Handle
 *      - the predicate index table is full.
 * NOTE: Does not apply the new predicate index to the atoms
 * inserted previously in the AtomTable.
 */
void PredicateIndex::addPredicateIndex(Handle ph,
                                       PredicateEvaluator* evaluator)
      throw (InvalidParamException)
{
	if (numberOfPredicateIndices > MAX_PREDICATE_INDICES)
	{
		throw InvalidParamException(TRACE_INFO,
			"AtomTable - Exceeded number of predicate indices = %d",
			MAX_PREDICATE_INDICES);
	}

	std::map<Handle, int>::iterator it;
	it = predicateHandles2Indices.find(ph);
	if (predicateHandles2Indices.end() != it)
	{
		throw InvalidParamException(TRACE_INFO,
			"AtomTable - There is already an index for predicate handle %p",
			ph.value());
	}

	// Ok, add it.
	predicateHandles2Indices.insert(
		std::pair<Handle,int>(ph, numberOfPredicateIndices));
	predicateHandles[numberOfPredicateIndices] = ph;
	predicateEvaluators[numberOfPredicateIndices] = evaluator;
	numberOfPredicateIndices++;
}

/**
 * Returns the Predicate evaluator for a given
 * GroundedPredicateNode Handle, if it is being used as a
 * lookup index. Otherwise, returns NULL.
 */
PredicateEvaluator* PredicateIndex::getPredicateEvaluator(Handle gpnHandle) const
{
	PredicateEvaluator* result = NULL;

	std::map<Handle, int>::const_iterator it;
	it = predicateHandles2Indices.find(gpnHandle);
	if (predicateHandles2Indices.end() != it)
	{
		int index = it->second;
		result = predicateEvaluators[index];
	}
	return result;
}

/**
 * Returns a list of handles that matches the GroundedPredicateNode
 * with the given Handle.
 * @param the Handle of the predicate node.
 * @param VersionHandle for filtering the resulting atoms by
 *       context. NULL_VERSION_HANDLE indicates no filtering
 **/
const UnorderedHandleSet& PredicateIndex::findHandlesByGPN(Handle gpnHandle) const
{
	static UnorderedHandleSet emptySet;
	if (!TLB::isValidHandle(gpnHandle)) return emptySet;

	std::map<Handle, int>::const_iterator it;
	it = predicateHandles2Indices.find(gpnHandle);
	if (it == predicateHandles2Indices.end()) return emptySet;

	return getHandleSet(it->second);
}

// ================================================================
