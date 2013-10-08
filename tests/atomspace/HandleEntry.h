/*
 * opencog/atomspace/HandleEntry.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_HANDLE_ENTRY_H
#define _OPENCOG_HANDLE_ENTRY_H

#include <unordered_set>
#include <string>
#include <vector>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/VersionHandle.h>

using std::string;

using namespace opencog;


/**
 * This class implements a linked-list of handles, and provides several
 * methods to perform operations between lists such as concatenation,
 * intersection, and filtering.
 */
class HandleEntry
{

public:

    static int existingObjects;

    /**
     * Cell dat
     */
    Handle handle;

    /**
     * Next cell.
     */
    HandleEntry *next;

    /**
     * Constructor for this class.
     *
     * @param Cell data.
     */
private:
    void init(Handle);
public:
    HandleEntry(Handle h) { init(h); }
    HandleEntry(Handle h, HandleEntry *he) {
        init(h);
        next = he;
    }

    /**
     * Destructor for this class.
     */
    ~HandleEntry();

    /**
     * Returns the atom referred by this HandleEntry using the TLB.
     *
     * @return Atom referred by this HandleEntry using the TLB.
     */
    AtomPtr getAtom();

    /**
     * Clones the list that starts in this handle entry cell, preserving
     * all cell data.
     *
     * @return Pointer to the new cloned list head.
     */
    HandleEntry* clone();

    /**
     * Returns the size of the linked-list from the current handle entry
     * head.
     *
     * @return Size of the linked-list from the current handle entry head.
     */
    int getSize();

    /**
     * Returns last member of the linked-list
     *
     * @return last member of the linked-list
     */
    HandleEntry* last();

    /**
     * Returns true iff this list contains the given handle.
     */
    bool contains(Handle h);

    /**
     * Returns a string representation of the list that starts in the
     * handle entry cell. The string returned is allocated in the
     * activation registry stack, so it must be duplicated if not
     * immediately used.
     *
     * @return A string representation of the list that starts in the
     * handle entry cell.
     */
    std::string toString();

    /**
     * Returns a vector of handles containing all data in the list that
     * starts in the handle entry cell.
     *
     * @return a vector of handles.
     */
    HandleSeq toHandleVector(void);

    /**
     * Returns a HandlEntry containing all data in the array passed
     * as a parameter.
     *
     * @param vector
     * @return A HandleEntry containing the elements of the vector.
     */
    static HandleEntry* fromHandleVector(const std::vector<Handle> &);
    static HandleEntry* fromHandleSet(const UnorderedHandleSet &);

    /**
     * Removes a handle from the list.
     * @param The linked list
     * @param The handle from atom that should be removed from the
     *        list
     * @return The filtered linked list
     */
    static HandleEntry* remove(HandleEntry*, Handle);

    /**
     * Returns the intersection between two linked-lists. The two
     * linked-lists passed as arguments are automatically destroyed.
     *
     * @param First linked-list.
     * @param Second linked-list.
     * @return The intersection between two linked-lists.
     */
    static HandleEntry* intersection(HandleEntry*, HandleEntry*);

    /**
     * Returns the intersection between several linked-lists. The n
     * linked-lists passed as arguments are automatically destroyed.
     *
     * @param Array of linked-lists.
     * @param Length of the array of linked-lists.
     * @return Intersection between the given linked-lists.
     */
    static HandleEntry* intersection(std::vector<HandleEntry*>&) throw (InconsistenceException);

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
    static int nextMatch(Handle**, int*, std::vector<int>&);

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
    static HandleEntry* intersection(Handle**, int*, int);

    /**
     * This method returns the concatenation of two linked-lists, the
     * first linked-list followed by the second.
     * THE FIRST LIST IS CHANGED AS SIDE-EFFECT if it is not null.
     *
     * @param First linked-list.
     * @param Second linked-list.
     * @return Concatenation of the first list with the second list.
     */
    static HandleEntry* concatenation(HandleEntry*, HandleEntry*);

    /**
     * Filters a linked-list. The filtering criterion is whether the atoms
     * in the linked-list belongs to a given AtomTable.
     *
     * @param Linked-list to be filtered.
     * @param AtomTableList indicating the AtomTable the atoms to be kept must belong
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, AtomTable *);

    /**
     * Filters a linked-list. The filtering criterion is whether the atoms
     * in the linked-list are marked for removal or not.
     *
     * @param Linked-list to be filtered.
     * @param Boolean indicating if the atoms to be kept must be flagged
     * for deletion or not.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, bool);

    /**
     * Filters a linked-list. The atoms kept in the resulting linked-list
     * exactly match their names to the passed key.
     *
     * @param Linked-list to be filtered.
     * @param Name key.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, const char*);

    /**
     * Filters a linked-list. The atoms kept have the same passed type, or
     * optionally are a subclass of it.
     *
     * @param Linked-list to be filtered.
     * @param Type key.
     * @param Whether the above type should consider subclasses as well.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, Type, bool);

    /**
     * Filters a linked-list. The atoms kept have the same name, same
     * passed type, or optionally are a subclass of it.
     *
     * @param Linked-list to be filtered.
     * @param Name key.
     * @param Type key.
     * @param Whether the above type should consider subclasses as well.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, const char*, Type, bool);

    /**
     * Filters a linked-list. The atoms kept have the given handle in their
     * outgoing set.
     *
     * @param Linked-list to be filtered.
     * @param Handle key.
     * @param Outgoing set cardinality.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, Handle, Arity);

    /**
     * Filters a linked-list. The atoms kept have the given arity
     *
     * @param Linked-list to be filtered.
     * @param Outgoing set cardinality.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, Arity);

    /**
     * Filters a linked-list. The atoms kept have the given handle in the
     * given position of their outgoing set.
     *
     * @param Linked-list to be filtered.
     * @param Handle key.
     * @param Outgoing set position to be matched.
     * @param Outgoing set cardinality.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, Handle, Arity, Arity);

    /**
     * Filters a linked-list. The atoms kept have atoms of the given type,
     * optionally its subclasses, in their outgoing set.
     *
     * @param Linked-list to be filtered.
     * @param Type key.
     * @param Whether the above type should consider subclasses as well.
     * @param Outgoing set cardinality.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, Type, bool, Arity);


    /**
     * Filters a linked-list. The atoms kept have atoms of the given type,
     * optionally its subclasses, in the given position of their outgoing
     * set.
     *
     * @param Linked-list to be filtered.
     * @param Type key.
     * @param Whether the above type should consider subclasses as well.
     * @param Outgoing set position to be matched.
     * @param Outgoing set cardinality.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, Type, bool, Arity, Arity);

    /**
     * Filters a linked-list. The atoms kept have atoms with both the name
     * and type, subclasses optionally, in the given position of their
     * outgoing set.
     *
     * @param Linked-list to be filtered.
     * @param Name key.
     * @param Type key.
     * @param Whether the above type should consider subclasses as well.
     * @param Outgoing set position to be matched.
     * @param Outgoing set cardinality.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, const char*, Type, bool, Arity, Arity);

    /**
     * Filters a linked-list. The atoms kept have atoms with the name
     * in the given position of their outgoing set.
     *
     * @param Linked-list to be filtered.
     * @param Name key.
     * @param Type key.
     * @param Whether the above type should consider subclasses as well.
     * @param Outgoing set position to be matched.
     * @param Outgoing set cardinality.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, const char*, Arity, Arity);

    /**
     * Filters a linked-list. The filtering criterion is the importance
     * range specified by the given boundaries.
     *
     * @param Linked-list to be filtered.
     * @param Importance range lower bound.
     * @param Importance range upper bound.
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, AttentionValue::sti_t, AttentionValue::sti_t);


    /**
     * Filters a linked-list. The atoms kept have versioned TVs associated
     * to the given VersionHandle.
     *
     * @param Linked-list to be filtered.
     * @param VersionHandle the remaining atoms must have
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, VersionHandle);

    /**
     * Filters a linked-list. The atoms kept have an atom of the given Type (or, optionally,
     * any subclass of it) in its outgoing set with a versioned TVs associated to the
     * given VersionHandle.
     *
     * @param Linked-list to be filtered. It must already have at least 1 target that
     * matches with the Type and bool arguments, i.e., this linked-list must be a result
     * from a buildSet operation that uses the corresponding target indices
     * @param the Type that the target atom must have
     * @param if the target atom can have any subclasses of the given Type argument
     * @param VersionHandle the matched atom at outgoing set must have
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, Type, bool, VersionHandle);

    /**
     * Filters a linked-list. The atoms kept have an atom of the given Name and Type in
     * its outgoing set with a versioned TVs associated to the given VersionHandle.
     *
     * @param Linked-list to be filtered. It must already have at least 1 target that
     * matches with the Name and Type arguments, i.e., this linked-list must be a result
     * from a buildSet operation that uses the corresponding target indices
     * @param the Name that the target atom must have
     * @param the Type that the target atom must have
     * @param VersionHandle the matched atom at outgoing set must have
     * @return Filtered linked-list.
     */
    static HandleEntry* filterSet(HandleEntry*, const char*, Type, VersionHandle);

    static bool matchesFilterCriteria(AtomPtr atom, Type targetType,
            bool targetSubclasses, VersionHandle vh);

    static bool matchesFilterCriteria(AtomPtr atom, const char* targetName,
            Type targetType, VersionHandle vh);
};


#endif // _OPENCOG_HANDLE_ENTRY_H
