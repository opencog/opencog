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

#if 0
    for (unsigned int band = 0; band < (unsigned int) IMPORTANCE_INDEX_SIZE; band++) {
        Handle current = importanceIndex[band];
        while (TLB::isValidHandle(current)) {
            Atom* atom = TLB::getAtom(current);
            Handle next = atom->next(IMPORTANCE_INDEX);

            // update sti
            atom->getAVPointer()->decaySTI();

            // update importanceIndex
            // potential optimization: if we may reliably assume that all atoms
            // decrease the sti by one unit (i.e. --sti), we could update the
            // indexes with "importanceIndex[band - 1] = importanceIndex[band];"
            updateImportance(atom, band);

            current = next;
        }
    }

    // cache the minSTI
    minSTI = config().get_int("MIN_STI");
    unsigned int lowerStiBand = importanceBin(minSTI);

    for (unsigned int band = 0; band <= lowerStiBand; band++) {
        Handle current = importanceIndex[band];
        while (TLB::isValidHandle(current)) {
            Atom* atom = TLB::getAtom(current);
            Handle next = atom->next(IMPORTANCE_INDEX);
            // remove it if too old
            if (atom->isOld(minSTI))
                oldAtoms = HandleEntry::concatenation(extractOld(current, true), oldAtoms);
            current = next;
        }
    }
#endif

	return oldAtoms;
}

// ================================================================
