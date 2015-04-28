/*
 * opencog/spacetime/TemporalTable.h
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

#ifndef _OPENCOG_TEMPORAL_TABLE_H
#define _OPENCOG_TEMPORAL_TABLE_H

#include <opencog/spacetime/HandleToTemporalEntryMap.h>
#include <opencog/spacetime/TemporalToHandleSetMap.h>
#include <opencog/spacetime/HandleTemporalPairEntry.h>

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

class TemporalTableFile;

// TODO: Depending on the use cases, this class would have a better performance
// if we use a sortedTemporalList in inverse cronological order.
// Or, if saving space is not required, we can even have 2 sortedTemporalLists.
class TemporalTable
{
    friend class TemporalTableFile;

public:

    /**
     * Temporal relationship criteria for lookups in this table, considering:
     *   T (a Temporal object inside this table) = (l,u), where l = lower bound, u = upper bound
     *   T'(a Temporal object argument used in the lookup criterion for both get and remove methods) = (l',u')
     *   R (the TemporalRelationship argument used in the lookup criterion), which is used as follows: T <R> T'
     * The lookup criteria definitions are as bellow:
     *  - EXACT               => T == T' => l == l' && u == u' && distribution type (normal or uniform) are the same
     *  - STARTS_BEFORE       => l < l'
     *  - STARTS_WITHIN       => l' <= l <= u'
     *  - STARTS_AFTER        => l > u'
     *  - ENDS_BEFORE         => u < l'
     *  - ENDS_WITHIN         => l' <= u <= u'
     *  - ENDS_AFTER          => u > u'
	 *  - OVERLAPS            => STARTS_WITHIN || ENDS_WITHIN => l' <= u && l <= u' (since l <= u && l' <= u') 
	 *  - INCLUDES            => l <= l' && u >= u'
     *  - NEXT_AFTER_START_OF => next time T whose l > l'
     *  - NEXT_AFTER_END_OF   => next time T whose l > u'
     *  - PREVIOUS_BEFORE_START_OF => previous time T whose l < l'
     *  - PREVIOUS_BEFORE_END_OF   => previous time T whose l < u'
     */
    typedef enum {
        EXACT,
        STARTS_BEFORE,
        STARTS_WITHIN,
        STARTS_AFTER,
        ENDS_BEFORE,
        ENDS_WITHIN,
        ENDS_AFTER,
        OVERLAPS, 
        INCLUDES, 
        NEXT_AFTER_START_OF,
        NEXT_AFTER_END_OF,
        PREVIOUS_BEFORE_START_OF,
        PREVIOUS_BEFORE_END_OF
    } TemporalRelationship;

    /**
     * Creates an empty TemporalTable.
     */
    TemporalTable();
    virtual ~TemporalTable();

    /**
     * Adds into this TemporalTable an entry composed by the given Atom Handle and Temporal object.
     */
    void add(Handle, const Temporal&);

    /**
     * Gets a list of HandleTemporalPair objects given an Atom Handle.
     * If the passed Handle object is Handle::UNDEFINED, it matches any Handle.
     * If the optional Temporal object argument is not UNDEFINED_TEMPORAL, it will be used
     * to restrict the return to only HandleTemporalPair objects whose Temporal
     * matches with it, according to the temporal relationship (search criteria) argument to
     * be applied with this given Temporal argument.
     * See the definition of TemporalRelationship enumeration to see the possible values for it.
     *
     * NOTE1: If the search criterion is of form NEXT_<CONDITION> or PREVIOUS_<CONDITION>
     *        there are 2 different situations:
     *        1) if the given Handle argument *is not* Handle::UNDEFINED, it will return at most 1 entry.
     *        2) if the given Handle argument *is* Handle::UNDEFINED, it may return many entries. In this case,
     *           all returned entries have the exact same Temporal value, but each one with a different handle.
     *
     * NOTE2: The caller must take care of deletion of the HandleTemporalPairEntry object.
     */
    HandleTemporalPairEntry* get(Handle, const Temporal& = UNDEFINED_TEMPORAL, TemporalRelationship = EXACT);

    /**
     * Removes HandleTemporalPair objects related to a given Atom Handle.
     * If the passed Handle object is Handle::UNDEFINED, it matches any Handle.
     * If the optional Temporal object argument is not UNDEFINED_TEMPORAL, it will be used
     * to restrict the removal to only HandleTemporalPair objects whose Temporal
     * matches with it, according to the temporal relationship (search criteria) argument to
     * be applied with this given Temporal argument.
     * See the definition of TemporalRelationship enumeration to see the possible values for it.
     * @return True if any entry corresponding to the given arguments was removed. False, otherwise.
     *
     * NOTE: If the search criterion is of form NEXT_<CONDITION> or PREVIOUS_<CONDITION>
     *       there are 2 different situations:
     *       1) if the given Handle argument *is not* Handle::UNDEFINED, it will remove at most 1 entry.
     *       2) if the given Handle argument *is* Handle::UNDEFINED, it may remove many entries. In this case,
     *          the removed entries have the exact same Temporal value, but each one with a different handle.
     *       This is not a typical removal case though!
     */
    bool remove(Handle, const Temporal& = UNDEFINED_TEMPORAL, TemporalRelationship = EXACT);


    /**
     * Gets the name of TimeNode that would represent this Temporal object
     */
    std::string getTimeNodeName() const;

    /**
     * Returns a string representation of the given Temporal relationship code.
     */
    static const char* getTemporalRelationshipStr(TemporalRelationship criterion);

private:

    HandleToTemporalEntryMap* handleMap;
    HandleToTemporalEntryMap* tailHandleMap;
    TemporalToHandleSetMap* temporalMap;

    TemporalEntry* sortedTemporalList;
    TemporalEntry** temporalIndexTable;
    int indexTableSize;
    int indexTableCount;
    int pendingUpdateCount;

    void addToMaps(Handle h, Temporal* t);
    void updateIndexTable(int numEntries);
    int getTemporalIndexTablePos(const Temporal&);
    int replaceIndexTablePosition(int pos, TemporalEntry* newEntry);
    TemporalEntry* getPreviousTemporalEntry(const Temporal&);
    void removeFromSortedEntries(const Temporal& t);
    HandleTemporalPairEntry* get(const Temporal& t, TemporalRelationship criterion = EXACT);
    bool remove(const Temporal& t, TemporalRelationship criterion = EXACT);
    bool matchesTimeCriterion(const Temporal& time, const Temporal& t, TemporalRelationship criterion, bool& searchFinished);

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_TEMPORAL_TABLE_H
