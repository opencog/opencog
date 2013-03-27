/*
 * opencog/atomspace/ImportanceIndex.h
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

#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/FixedIntegerIndex.h>

namespace opencog
{
class Atom;
class AtomTable;
class HandleEntry;

/**
 * Implements an index with additional routines needed for managing 
 * short-term importance.
 */
class ImportanceIndex: public FixedIntegerIndex
{
private:
    static HandleEntry* extractOld(const AtomTable*, AttentionValue::sti_t,
		                             Handle, bool recursive = false);
public:
    ImportanceIndex(void);
    void insertAtom(const Atom*);
    void removeAtom(const Atom*);

    /** Updates the importance index for the given atom.
     * According to the new importance of the atom, it may change importance
     * bins.
     *
     * @param The atom whose importance index will be updated.
     * @param The old importance bin where the atom originally was.
     */
    void updateImportance(Atom*, int);

    HandleEntry* decayShortTermImportance(const AtomTable*);
    HandleEntry* getHandleSet(AttentionValue::sti_t,
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

    //! Tests whether an atom should be considered "old"
    static bool isOld(const Atom* a, const AttentionValue::sti_t threshold);

};

} //namespace opencog

#endif // _OPENCOG_IMPORTANCEINDEX_H
