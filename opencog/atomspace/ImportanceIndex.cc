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

#include <opencog/atomspace/ImportanceIndex.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/util/Config.h>

using namespace opencog;

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

void ImportanceIndex::updateImportance(Atom* atom, int bin)
{
	Handle h = atom->getHandle();
	int newbin = importanceBin(atom->getAttentionValue().getSTI());
	if (bin == newbin) return;

	remove(bin, h);
	insert(newbin, h);
}

void ImportanceIndex::insertAtom(const Atom* atom)
{
	int sti = atom->getAttentionValue().getSTI();
	int bin = importanceBin(sti);
	insert(bin, atom->getHandle());
}

void ImportanceIndex::removeAtom(const Atom* atom)
{
	int sti = atom->getAttentionValue().getSTI();
	int bin = importanceBin(sti);
	remove(bin, atom->getHandle());
}

UnorderedHandleSet ImportanceIndex::decayShortTermImportance(const AtomTable* atomtable)
{
	UnorderedHandleSet oldAtoms;

	unsigned int bin;
	if (IMPORTANCE_INDEX_SIZE != (1 << 16) ) {
       for (bin = 0; bin < IMPORTANCE_INDEX_SIZE; bin++) {
            UnorderedHandleSet move_it;
            UnorderedHandleSet& band = idx[bin];
            UnorderedHandleSet::iterator hit;
            for (hit = band.begin(); hit != band.end(); ++hit) {
                Handle h = *hit;
                Atom *atom = atomtable->getAtom(h);

                atom->attentionValue.decaySTI();
                unsigned int newbin = importanceBin(atom->attentionValue.getSTI());
                if (newbin != bin) {
                    insert(newbin, h);
                    move_it.insert(h);
                }
            }
            for (hit = move_it.begin(); hit != move_it.end(); ++hit) {
                remove(bin, *hit);
            }
        }
    } else {
        // Update STI
        // Optimization for the case where there is a bin for EVERY possible
        // importance value
        //
        // Assumes that all atoms decrease the sti by one unit 
        // (i.e. --sti), we could update the indexes with:
        // "importanceIndex[band - 1] = importanceIndex[band];"

        UnorderedHandleSet & band = idx[1];
        for (UnorderedHandleSet::iterator hit = band.begin(); hit != band.end(); hit++) {
            Handle h = *hit;
            Atom *atom = atomtable->getAtom(h);
            atom->attentionValue.decaySTI();
        }
        idx[0].insert(idx[1].begin(), idx[1].end());
        for (bin = 1; bin < IMPORTANCE_INDEX_SIZE-1; bin++)
        {
            idx[bin] = idx[bin+1];
            UnorderedHandleSet & band = idx[bin];
            for (UnorderedHandleSet::iterator hit = band.begin(); hit != band.end(); hit++) {
                Handle h = *hit;
                Atom *atom = atomtable->getAtom(h);
                atom->attentionValue.decaySTI();
            }
        }
        idx[bin].clear();
    }

	AttentionValue::sti_t minSTI = config().get_int("MIN_STI");
	unsigned int lowerStiBand = importanceBin(minSTI);

	for (bin = 0; bin <= lowerStiBand; bin++)
	{
		UnorderedHandleSet::iterator hit;
		UnorderedHandleSet & band = idx[bin];
		for (hit = band.begin(); hit != band.end(); ++hit) {
			Atom *atom = atomtable->getAtom(*hit);

			// Remove it if too old.
			if (isOld(atom,minSTI)) {
				UnorderedHandleSet un = extractOld(atomtable, minSTI, *hit, true);
            oldAtoms.insert(un.begin(), un.end());
         }
		}
	}

	return oldAtoms;
}

bool ImportanceIndex::isOld(const Atom* atom,
        const AttentionValue::sti_t threshold)
{
    return ((atom->getAttentionValue().getSTI() < threshold) &&
            (atom->getAttentionValue().getLTI() < 1));
}

UnorderedHandleSet ImportanceIndex::extractOld(const AtomTable* atomtable,
                                         AttentionValue::sti_t minSTI,
                                         Handle handle, bool recursive)
{
   UnorderedHandleSet result;
	Atom *atom = atomtable->getAtom(handle);

	if (atom->getFlag(REMOVED_BY_DECAY)) return result;
	else atom->setFlag(REMOVED_BY_DECAY, true);

	// If recursive-flag is set, also extract all the links in the
	// atom's incoming set.
	if (recursive) {
		const UnorderedHandleSet& iset = atomtable->getIncomingSet(handle);
		for (UnorderedHandleSet::const_iterator it = iset.begin();
		     it != iset.end(); it++)
		{
			Atom* a = atomtable->getAtom(*it);

			// XXX a should never be NULL; however, someone somewhere is
			// removing atoms from the TLB, although they forgot to remove
			// them from the atomTable, and so they are still showing up
			// i the incoming set. XXX TODO uncomment the assert below.
			// OC_ASSERT(a, "Atom removed from TLB But its still in the atomtable!");
			if (a and isOld(a, minSTI)) {
				UnorderedHandleSet un = extractOld(atomtable, minSTI, *it, true);
            result.insert(un.begin(), un.end());
			}
		}
	}

	// Only return if there is at least one incoming atom that is
	// not marked for removal by decay.
	const UnorderedHandleSet& iset = atomtable->getIncomingSet(handle);
	for (UnorderedHandleSet::const_iterator it = iset.begin();
	        it != iset.end(); it++)
	{
		Atom* a = atomtable->getAtom(*it);
		// XXX a should never be NULL; however, someone somewhere is
		// removing atoms from the TLB, although they forgot to remove
		// them from the atomTable, and so they are still showing up
		// i the incoming set. XXX TODO uncomment the assert below.
		// OC_ASSERT(a, "Atom removed from TLB But its still in the atomtable!");
		if (a and a->getFlag(REMOVED_BY_DECAY) == false) {
			atom->setFlag(REMOVED_BY_DECAY, false);
			return result;
		}
	}
   result.insert(handle);
	return result;
}

UnorderedHandleSet ImportanceIndex::getHandleSet(
        const AtomTable* atomtable,
        AttentionValue::sti_t lowerBound,
        AttentionValue::sti_t upperBound) const
{
	UnorderedHandleSet set;

	// The indexes for the lower bound and upper bound lists is returned.
	int lowerBin = importanceBin(lowerBound);
	int upperBin = importanceBin(upperBound);

	// Build a list of atoms whose importance is equal to the lower bound.
	const UnorderedHandleSet &sl = idx[lowerBin];

	// For the lower bound and upper bound index, the list is filtered,
	// because there may be atoms that have the same importanceIndex
	// and whose importance is lower than lowerBound or bigger than
	// upperBound.
	std::copy_if(sl.begin(), sl.end(), inserter(set),
		[&](Handle h)->bool {
			AttentionValue::sti_t sti = 
				atomtable->getAtom(h)->getAttentionValue().getSTI();
			return (lowerBound <= sti and sti <= upperBound);
		});
  
	if (lowerBin == upperBin) {
		// If both lower and upper bounds are in the same bin,
		// Then we are done.
		return set;
	}

	// For every index within lowerBound and upperBound,
	// add to the list.
	while (++lowerBin < upperBin) {
		const UnorderedHandleSet &ss = idx[lowerBin];
		set.insert(ss.begin(), ss.end());
	}

	// The two lists are concatenated.
	const UnorderedHandleSet &uset = idx[upperBin];
	std::copy_if(uset.begin(), uset.end(), inserter(set),
		[&](Handle h)->bool {
			AttentionValue::sti_t sti = 
				atomtable->getAtom(h)->getAttentionValue().getSTI();
			return (lowerBound <= sti and sti <= upperBound);
		});

	return set;
}

