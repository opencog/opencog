/*
 * opencog/atomspace/ImportanceIndex.cc
 *
 * Copyright (C) 2008-2011 OpenCog Foundation
 * All Rights Reserved
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

#include <algorithm>

#include <opencog/util/Config.h>
#include <opencog/util/functional.h>
#include <opencog/util/oc_assert.h>

#include <opencog/atomspace/ImportanceIndex.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/AtomTable.h>

using namespace opencog;

//! 4092 importance bins means each bin has STI range of 32
#define IMPORTANCE_INDEX_SIZE   (1 << 11)

ImportanceIndex::ImportanceIndex(void)
{
	resize(IMPORTANCE_INDEX_SIZE+1);
}

unsigned int ImportanceIndex::importanceBin(short importance)
{
	// STI is in range of [-32768, 32767] so adding 32768 puts it in
	// [0, 65535]
	return (importance + 32768) / IMPORTANCE_INDEX_SIZE;
}

void ImportanceIndex::updateImportance(AtomPtr atom, int bin)
{
	Handle h = atom->getHandle();
	int newbin = importanceBin(atom->getAttentionValue()->getSTI());
	if (bin == newbin) return;

	remove(bin, h);
	insert(newbin, h);
}

void ImportanceIndex::insertAtom(AtomPtr atom)
{
	int sti = atom->getAttentionValue()->getSTI();
	int bin = importanceBin(sti);
	insert(bin, atom->getHandle());
}

void ImportanceIndex::removeAtom(AtomPtr atom)
{
	int sti = atom->getAttentionValue()->getSTI();
	int bin = importanceBin(sti);
	remove(bin, atom->getHandle());
}

UnorderedHandleSet ImportanceIndex::getHandleSet(
        const AtomTable* atomtable,
        AttentionValue::sti_t lowerBound,
        AttentionValue::sti_t upperBound) const
{
	UnorderedUUIDSet set;

	// The indexes for the lower bound and upper bound lists is returned.
	int lowerBin = importanceBin(lowerBound);
	int upperBin = importanceBin(upperBound);

	// Build a list of atoms whose importance is equal to the lower bound.
	const UnorderedUUIDSet &sl = idx[lowerBin];

	// For the lower bound and upper bound index, the list is filtered,
	// because there may be atoms that have the same importanceIndex
	// and whose importance is lower than lowerBound or bigger than
	// upperBound.
	std::copy_if(sl.begin(), sl.end(), inserter(set),
		[&](UUID uuid)->bool {
			Handle h(uuid);
			AttentionValue::sti_t sti =
				h->getAttentionValue()->getSTI();
			return (lowerBound <= sti and sti <= upperBound);
		});

	if (lowerBin == upperBin) {
		// If both lower and upper bounds are in the same bin,
		// Then we are done.
		UnorderedHandleSet ret;
		std::transform(set.begin(), set.end(), inserter(ret),
			[](UUID uuid)->Handle { return Handle(uuid); });
		return ret;
	}

	// For every index within lowerBound and upperBound,
	// add to the list.
	while (++lowerBin < upperBin) {
		const UnorderedUUIDSet &ss = idx[lowerBin];
		set.insert(ss.begin(), ss.end());
	}

	// The two lists are concatenated.
	const UnorderedUUIDSet &uset = idx[upperBin];
	std::copy_if(uset.begin(), uset.end(), inserter(set),
		[&](UUID uuid)->bool {
			Handle h(uuid);
			AttentionValue::sti_t sti = h->getAttentionValue()->getSTI();
			return (lowerBound <= sti and sti <= upperBound);
		});

	UnorderedHandleSet ret;
	std::transform(set.begin(), set.end(), inserter(ret),
		[](UUID uuid)->Handle { return Handle(uuid); });
	return ret;
}

