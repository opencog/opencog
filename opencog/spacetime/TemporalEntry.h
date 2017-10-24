/*
 * opencog/spacetime/TemporalEntry.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Carlos Lopes <dlopes@vettalabs.com>
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

#ifndef _OPENCOG_TEMPORAL_ENTRY_H
#define _OPENCOG_TEMPORAL_ENTRY_H

#include <string>
#include <vector>

#include <stdlib.h>
#include <limits.h>

#include <opencog/spacetime/Temporal.h>

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

/**
 * This class implements a linked-list of times, and provides several
 * methods to perform operations between lists such as concatenation,
 * intersection, and filtering.
 */
class TemporalEntry
{

public:

    static int existingObjects;

    /**
     * Cell dat
     */
    Temporal* time;

    /**
     * Next cell.
     */
    TemporalEntry *next;

    /**
     * Constructor for this class.
     *
     * @param Cell data.
     */
    TemporalEntry(Temporal*);

    /**
     * Destructor for this class.
     */
    ~TemporalEntry();

    /**
     * Clones the list that starts in this time entry cell, preserving
     * all cell data.
     *
     * @return Pointer to the new cloned list head.
     */
    TemporalEntry* clone();

    /**
     * Returns the size of the linked-list from the current time entry
     * head.
     *
     * @return Size of the linked-list from the current time entry head.
     */
    int getSize();

    /**
     * Returns last member of the linked-list
     *
     * @return last member of the linked-list
     */
    TemporalEntry* last();

    /**
     * Returns true iff this list contains the given time.
     */
    bool contains(Temporal*);

    /**
     * Returns a string representation of the list that starts in the
     * time entry cell. The string returned is allocated in the
     * activation registry stack, so it must be duplicated if not
     * immediately used.
     *
     * @return A string representation of the list that starts in the
     * time entry cell.
     */
    std::string toString();

    /**
     * Returns a vector of times containing all data in the list that
     * starts in the time entry cell.
     *
     * @param Pointer passed by reference where the vector will be
     * allocated.
     * @param Length of the allocated vector passed by reference.
     * @return Pointer passed by reference where the vector will be
     * allocated. Same as the first parameter.
     */
    Temporal** toTemporalVector(Temporal**&, int&);

    /**
     * Returns a HandlEntry containing all data in the array passed
     * as a parameter.
     *
     * @param Pointer where the vector is stored.
     * @param Length of the allocated vector.
     * @return A TemporalEntry containing the element s of the vector.
     */
    static TemporalEntry* fromTemporalVector(Temporal**, int);

    /**
     * Adds a time to the list.
     * @param The linked list
     * @param The time that should be added to the list
     * @return The resulting linked list after adding the time
     */
    static TemporalEntry* add(TemporalEntry*, Temporal*);

    /**
     * Removes a time from the list.
     * @param The linked list
     * @param The time from atom that should be removed from the
     *        list
     * @return The filtered linked list
     */
    static TemporalEntry* remove(TemporalEntry*, Temporal*);

    /**
     * Returns the intersection between two linked-lists. The two
     * linked-lists passed as arguments are automatically destroyed.
     *
     * @param First linked-list.
     * @param Second linked-list.
     * @return The intersection between two linked-lists.
     */
    static TemporalEntry* intersection(TemporalEntry*, TemporalEntry*);

    /**
     * Returns the intersection between several linked-lists. The n
     * linked-lists passed as arguments are automatically destroyed.
     *
     * @param Array of linked-lists.
     * @param Length of the array of linked-lists.
     * @return Intersection between the given linked-lists.
     */
    static TemporalEntry* intersection(TemporalEntry**, int);

    /**
     * This method is internal for the intersection calculation methods.
     * It returns the position of a common element in the first array if
     * the element is present in all arrays.
     *
     * @param Array of vector where each represents a linked-list.
     * @param Array of vector lengths.
     * @param Internal data structure that represents the current
     * searching position.
     * @return The position of a common element in the first array.
     */
    static int nextMatch(Temporal***, int*, std::vector<int>&);

    /**
     * This method is internal for the intersection calculation methods.
     * It returns the intersection of an array of vectors where each
     * represents a linked-list.
     *
     * @param Array of vector where each represents a linked-list.
     * @param Array of vector lengths.
     * @param Number of vectors.
     * @return Intersection between the given linked-lists.
     */
    static TemporalEntry* intersection(Temporal***, int*, int);

    /**
     * This method returns the concatenation of two linked-lists, the
     * first linked-list followed by the second.
     * THE FIRST LIST IS CHANGED AS SIDE-EFFECT if it is not null.
     *
     * @param First linked-list.
     * @param Second linked-list.
     * @return Concatenation of the first list with the second list.
     */
    static TemporalEntry* concatenation(TemporalEntry*, TemporalEntry*);

    /**
     * Temporal* sort criterion used by qsort. It returns a negative value,
     * zero or a positive value if the first argument is respectively
     * smaller than, equal to, or larger then the second argument.
     *
     * @param The first time element.
     * @param The second time element.
     * @return A negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger then the
     * second argument.
     */
    static int temporalCompare(const void*, const void*);

    /**
     * Temporal* sort criterion used by qsort. It returns a negative value,
     * zero or a positive value if the first argument is respectively
     * smaller than, equal to, or larger then the second argument.
     *
     * @param The first time element.
     * @param The second time element.
     * @return A negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger then the
     * second argument.
     */
    static int compare(const Temporal*, const Temporal*);

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_TEMPORAL_ENTRY_H
