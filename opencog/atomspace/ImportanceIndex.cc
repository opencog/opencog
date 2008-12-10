/*
 * opencog/atomspace/ImportanceIndex.cc
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

#include <opencog/atomspace/ImportanceIndex.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/util/Config.h>

using namespace opencog;

ImportanceIndex::ImportanceIndex(void)
{
	resize(IMPORTANCE_INDEX_SIZE);
}

/**
 * This method returns which importance bin an atom with the given
 * importance should be placed.
 *
 * @param Importance value to be mapped.
 * @return The importance bin which an atom of the given importance
 * should be placed.
 */
unsigned int ImportanceIndex::importanceBin(short importance)
{
	// STI is in range of [-32768, 32767] so adding 32768 puts it in
	// [0, 65535] which is the size of the index
	return importance + 32768;
}

/**
 * Returns the mean importance value for the given importance bin (the
 * average between the lower and upper importance bounds for the bin).
 *
 * @param Importance bin to be mapped.
 * @return The mean importance value for the given importance bin.
 */
float ImportanceIndex::importanceBinMeanValue(unsigned int bin)
{
	return (((float) bin) + 0.5) / ((float) IMPORTANCE_INDEX_SIZE);
}

/**
 * Updates the importance index for the given atom. According to the
 * new importance of the atom, it may change importance bins.
 *
 * @param The atom whose importance index will be updated.
 * @param The old importance bin where the atom originally was.
 */
void ImportanceIndex::updateImportance(Atom* atom, int bin)
{
	Handle h = TLB::getHandle(atom);
	remove(bin, h);

	int newbin = importanceBin(atom->getAttentionValue().getSTI());
	insert(newbin, h);
}

HandleEntry* ImportanceIndex::decayShortTermImportance(void)
{
	HandleEntry* oldAtoms = NULL;

	std::vector<std::set<Handle> >::iterator band;
	unsigned int bin;
	for (bin = 0, band = idx.begin(); band != idx.end(); band++, bin++)
	{
		std::set<Handle>::iterator hit;
		for (hit = band->begin(); hit != band->end(); hit++)
		{
			Atom *atom = TLB::getAtom(*hit);

			// Update sti
			atom->getAVPointer()->decaySTI();

			// Potential optimization: if we may reliably assume that all atoms
			// decrease the sti by one unit (i.e. --sti), we could update the
			// indexes with "importanceIndex[band - 1] = importanceIndex[band];"
			updateImportance(atom, bin);
		}
	}

	AttentionValue::sti_t minSTI = config().get_int("MIN_STI");
	unsigned int lowerStiBand = importanceBin(minSTI);

	for (bin = 0, band = idx.begin(); bin <= lowerStiBand; band++, bin++)
	{
		std::set<Handle>::iterator hit;
		for (hit = band->begin(); hit != band->end(); hit++)
		{
			Atom *atom = TLB::getAtom(*hit);

			// Remove it if too old.
			if (atom->isOld(minSTI))
				oldAtoms = HandleEntry::concatenation(
					extractOld(minSTI, *hit, true), oldAtoms);
		}
	}

	return oldAtoms;
}

HandleEntry* ImportanceIndex::extractOld(AttentionValue::sti_t minSTI,
                                         Handle handle, bool recursive)
{
	HandleEntry* result = NULL;
	Atom *atom = TLB::getAtom(handle);

	if (atom->getFlag(REMOVED_BY_DECAY)) return result;
	else atom->setFlag(REMOVED_BY_DECAY, true);

	// If recursive-flag is set, also extract all the links in the
	// atom's incoming set.
	if (recursive)
	{
		for (HandleEntry* in = atom->getIncomingSet(); in != NULL; in = in->next)
		{
			Atom *a = TLB::getAtom(in->handle);
			if (a->isOld(minSTI))
			{
				result = HandleEntry::concatenation(
					extractOld(minSTI, in->handle, true), result);
			}
		}
	}

	// Only return if there is at least one incoming atom that is
	// not marked for removal by decay.
	for (HandleEntry* in = atom->getIncomingSet(); in != NULL; in = in->next)
	{
		if (TLB::getAtom(in->handle)->getFlag(REMOVED_BY_DECAY) == false)
		{
			atom->setFlag(REMOVED_BY_DECAY, false);
			return result;
		}
	}
	result = HandleEntry::concatenation(new HandleEntry(handle), result);
	return result;
}

HandleEntry* 
ImportanceIndex::getHandleSet(AttentionValue::sti_t lowerBound,
                              AttentionValue::sti_t upperBound) const
{
	HandleEntry *set = NULL;

	// The indexes for the lower bound and upper bound lists is returned.
	int lowerBin = importanceBin(lowerBound);
	int upperBin = importanceBin(upperBound);

	// Build a list of atoms whose importance is equal to the lower bound.
	std::set<Handle>::const_iterator hit;
	const std::set<Handle> &sl = idx[lowerBin];
	for (hit = sl.begin(); hit != sl.end(); hit++)
	{
		HandleEntry *he = new HandleEntry(*hit);
		he->next = set;
		set = he;
	}

	// For the lower bound and upper bound index, the list is filtered,
	// because there may be atoms that have the same importanceIndex
	// and whose importance is lower than lowerBound or bigger than
	// upperBound.  XXX This doesn't sound right. Maybe after being 
	// decayed ... but this should not be generally true.
	set = HandleEntry::filterSet(set, lowerBound, upperBound);

	if (lowerBin == upperBin)
	{
		// If both lower and upper bounds are in the same bin,
		// Then we are done.
		return set;
	}

	// For every index within lowerBound and upperBound,
	// add to the list.
	while (++lowerBin < upperBin)
	{
		const std::set<Handle> &ss = idx[lowerBin];
		for (hit = ss.begin(); hit != ss.end(); hit++)
		{
			HandleEntry *he = new HandleEntry(*hit);
			he->next = set;
			set = he;
		}
	}

	HandleEntry *uset = NULL;
	const std::set<Handle> &su = idx[upperBin];
	for (hit = su.begin(); hit != su.end(); hit++)
	{
		HandleEntry *he = new HandleEntry(*hit);
		he->next = uset;
		uset = he;
	}
	uset = HandleEntry::filterSet(uset, lowerBound, upperBound);

	// The two lists are concatenated.
	return HandleEntry::concatenation(uset, set);
}

// ================================================================
