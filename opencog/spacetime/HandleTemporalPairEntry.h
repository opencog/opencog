/*
 * opencog/spacetime/HandleTemporalPairEntry.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#ifndef _OPENCOG_HANDLE_TEMPORAL_PAIR_ENTRY_H
#define _OPENCOG_HANDLE_TEMPORAL_PAIR_ENTRY_H

#include <string>
#include <vector>

#include <stdlib.h>
#include <limits.h>

#include <opencog/util/exceptions.h>

#include <opencog/spacetime/HandleTemporalPair.h>

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

/**
 * This class implements a linked-list of HandleTemporalPair objects, and provides several
 * methods to perform operations between lists such as concatenation.
 */
class HandleTemporalPairEntry
{

private:
    /**
     * HandleTemporalPair sort criterion used by qsort. It returns a negative value,
     * zero or a positive value if the first argument is respectively
     * smaller than, equal to, or larger then the second argument.
     *
     * @param The first handleTemporalPair element.
     * @param The second handleTemporalPair element.
     * @return A negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger then the
     * second argument.
     */
    static int compare(const HandleTemporalPair&, const HandleTemporalPair&);

public:

    static int existingObjects;

    /**
     * Cell dat
     */
    HandleTemporalPair handleTemporalPair;

    /**
     * Next cell.
     */
    HandleTemporalPairEntry *next;

    /**
     * Constructor for this class.
     *
     * @param Cell data.
     */
    HandleTemporalPairEntry(const HandleTemporalPair&);

    /**
     * Constructor for this class.
     *
     * @param Cell data.
     */
    HandleTemporalPairEntry(Handle, Temporal*);

    /**
     * Destructor for this class.
     */
    ~HandleTemporalPairEntry();

    /**
     * Clones the list that starts in this handleTemporalPair entry cell, preserving
     * all cell data.
     *
     * @return Pointer to the new cloned list head.
     */
    HandleTemporalPairEntry* clone();

    /**
     * Returns the size of the linked-list from the current handleTemporalPair entry
     * head.
     *
     * @return Size of the linked-list from the current handleTemporalPair entry head.
     */
    int getSize();

    /**
     * Returns last member of the linked-list
     *
     * @return last member of the linked-list
     */
    HandleTemporalPairEntry* last();

    /**
     * Returns true iff this list contains the given handleTemporalPair.
     */
    bool contains(const HandleTemporalPair&);

    /**
     * Returns a string representation of the list that starts in the
     * handleTemporalPair entry cell. The string returned is allocated in the
     * activation registry stack, so it must be duplicated if not
     * immediately used.
     *
     * @return A string representation of the list that starts in the
     * handleTemporalPair entry cell.
     */
    std::string toString();

    /**
     * Returns a vector of handleTemporalPairs containing all data in the list that
     * starts in the handleTemporalPair entry cell.
     *
     * @param Pointer passed by reference where the vector will be
     * allocated.
     * @param Length of the allocated vector passed by reference.
     * @return Pointer passed by reference where the vector will be
     * allocated. Same as the first parameter.
     */
    HandleTemporalPair* toHandleTemporalPairVector(HandleTemporalPair*&, int&);

    /**
     * Returns a HandlEntry containing all data in the array passed
     * as a parameter.
     *
     * @param Pointer where the vector is stored.
     * @param Length of the allocated vector.
     * @return A HandleTemporalPairEntry containing the element s of the vector.
     */
    static HandleTemporalPairEntry* fromHandleTemporalPairVector(HandleTemporalPair*, int);

    /**
     * Adds a handleTemporalPair to the list.
     * @param The linked list
     * @param The handleTemporalPair that should be added to the list
     * @param Flat to indicate if it must keep the sorting
     * @return The resulting linked list after adding the handleTemporalPair
     * NOTE: The handleTemporalPair argument to be added will be cloned internally.
     * So, the caller must take care of delete such argument when it is not used anymore.
     */
    static HandleTemporalPairEntry* add(HandleTemporalPairEntry*, const HandleTemporalPair&);

    /**
     * Removes a handleTemporalPair from the list.
     * @param The linked list
     * @param The handleTemporalPair from atom that should be removed from the
     *        list
     * @return The filtered linked list
     */
    static HandleTemporalPairEntry* remove(HandleTemporalPairEntry*, const HandleTemporalPair&);

    /**
     * Returns the intersection between two linked-lists. The two
     * linked-lists passed as arguments are automatically destroyed.
     *
     * @param First linked-list.
     * @param Second linked-list.
     * @return The intersection between two linked-lists.
     */
    static HandleTemporalPairEntry* intersection(HandleTemporalPairEntry*, HandleTemporalPairEntry*);

    /**
     * Returns the intersection between several linked-lists. The n
     * linked-lists passed as arguments are automatically destroyed.
     *
     * @param Array of linked-lists.
     * @param Length of the array of linked-lists.
     * @return Intersection between the given linked-lists.
     */
    static HandleTemporalPairEntry* intersection(HandleTemporalPairEntry**, unsigned int);

    /**
     * This method is internal for the intersection calculation methods.
     * It returns the position of a common element in the first array if
     * the element is present in all arrays.
     *
     * @param Array of vector where each represents a linked-list.
     * @param Internal data structure that represents the current
     * searching position.
     * @return The position of a common element in the first array.
     */
    static int nextMatch(std::vector<std::vector<HandleTemporalPair> >&, std::vector<unsigned int>&);

    /**
     * This method is internal for the intersection calculation methods.
     * It returns the intersection of an array of vectors where each
     * represents a linked-list.
     *
     * @param Array of vector where each represents a linked-list.
     * @return Intersection between the given linked-lists.
     */
    static HandleTemporalPairEntry* intersection(std::vector<std::vector<HandleTemporalPair> >&);

    /**
     * This method returns the concatenation of two linked-lists, the
     * first linked-list followed by the second.
     * THE FIRST LIST IS CHANGED AS SIDE-EFFECT if it is not null.
     *
     * @param First linked-list.
     * @param Second linked-list.
     * @return Concatenation of the first list with the second list.
     */
    static HandleTemporalPairEntry* concatenation(HandleTemporalPairEntry*, HandleTemporalPairEntry*);

    /**
     * HandleTemporalPair sort criterion used by qsort. It returns a negative value,
     * zero or a positive value if the first argument is respectively
     * smaller than, equal to, or larger then the second argument.
     *
     * @param The first handleTemporalPair element.
     * @param The second handleTemporalPair element.
     * @return A negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger then the
     * second argument.
     */
    static int handleTemporalPairCompare(const void*, const void*);

    class SortComparison
    {
    public:
        bool operator()(const HandleTemporalPair& htp1, const HandleTemporalPair& htp2) const {
            return(compare(htp1, htp2) < 0);
        }
    };

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_HANDLE_TEMPORAL_PAIR_ENTRY_H
