/*
 * opencog/attentionbank/ImportanceIndex.h
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

#ifndef _OPENCOG_IMPORTANCEINDEX_H
#define _OPENCOG_IMPORTANCEINDEX_H

#include <mutex>
#include <opencog/truthvalue/AttentionValue.h>
#include <opencog/attentionbank/ThreadSafeFixedIntegerIndex.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class AttentionBank;
/**
 * Implements an index with additional routines needed for managing
 * short-term importance.  This index is thread-safe.
 */
using HandleSTIPair = std::pair<Handle,AttentionValue::sti_t>;
class ImportanceIndex
{
private:
    AttentionBank& _bank;
    ThreadSafeFixedIntegerIndex _index;
    std::vector<HandleSTIPair> topKSTIValuedHandles; // TOP K STI values
    std::mutex topKSTIUpdateMutex;
    int minAFSize;

    void updateTopStiValues(Atom* atom);

public:
    ImportanceIndex(AttentionBank&);
    void removeAtom(Atom*, int);

    /**
     * Updates the importance index for the given atom.
     * According to the new importance of the atom, it may change importance
     * bins.
     *
     * @param The atom whose importance index will be updated.
     * @param The old importance bin where the atom originally was.
     */
    void updateImportance(Atom*, int, int);

    UnorderedHandleSet getHandleSet(AttentionValue::sti_t,
                                    AttentionValue::sti_t) const;

    /**
     * This method returns which importance bin an atom with the given
     * importance should be placed.
     *
     * @param Importance value to be mapped.
     * @return The importance bin which an atom of the given importance
     * should be placed.
     */
    static unsigned int importanceBin(short);

    /**
     * Get the content of an ImportanceBin at index i.
    */
    template <typename OutputIterator> OutputIterator
    getContent(size_t i,OutputIterator out) const
    {
            return _index.getContent(i,out);
    }

    Handle getRandomAtom(void)
    {
        return _index.getRandomAtom();
    }

    /**
     * Get the highest bin which contains Atoms
     */
    UnorderedHandleSet getMaxBinContents();

    /**
     * Get the lowest bin which contains Atoms
     */
    UnorderedHandleSet getMinBinContents();
    
    /**
     * Get latest top K sti values.
     */
     HandleSeq getTopSTIValuedHandles();

    size_t bin_size(void) const;
    
    /**
     * Get the size of the bin at the given index.
     */
    size_t size(int) const;
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_IMPORTANCEINDEX_H
