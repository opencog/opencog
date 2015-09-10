/*
 * opencog/embodiment/Learning/behavior/CompositeBehaviorDescription.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef COMPOSITEBEHAVIORDESCRIPTION_H
#define COMPOSITEBEHAVIORDESCRIPTION_H

#include <vector>
#include <opencog/atomspace/types.h>
#include <opencog/spacetime/Temporal.h>
#include "PredicateHandleSet.h"
#include "ElementaryBehaviorDescription.h"

namespace behavior
{

class CompositeBehaviorDescription
{

public:

    // ***********************************************/
    // Constructors/destructors

    /**
     * Build an empty CompositeBehaviorDescription. One must use the building
     * API to actually build a non-empty CompositeBehaviorDescription.
     *
     * Take a look at the document here:
     * http://wiki.opencog.org/w/BehaviorSimilarityAlgorithm_%28Embodiment%29
     * for a more precise definition of CompositeBehaviorDescription.
     */
    CompositeBehaviorDescription(AtomSpace *_atomspace);

    ~CompositeBehaviorDescription();

    // ***********************************************/
    // Building API

    /**
     * Add a pair (H, T) to this BehaviorDescriptor. A BehaviorDescriptor is,
     * in fact, a set of such pairs.
     *
     * @param handle The handle of the predicate node
     * @param interval A time interval where the passed predicates is valid
     */
    void addPredicate(Handle handle, const Temporal &interval);

    //this method is equivalent to above and provided only for convenience
    void addPredicate(Handle handle, unsigned long start_time, unsigned long end_time);

    //this method is equivalent to above and provided only for convenience
    void addPredicate(const ElementaryBehaviorDescription& ebd);

    // ***********************************************/
    // Manipulation API

    /**
     * @return Set of sets in the timeline representation of this CompositeBehaviorDescription
     */
    const std::vector<PredicateHandleSet> &getTimelineSets() const;

    /**
     * @return Set of time lengths in the timeline representation of this CompositeBehaviorDescription
     */
    const std::vector<long> &getTimelineIntervals() const;

    /**
     * @return the start time corresponding to a given index of the timeline representation
     */
    unsigned long getIndexStartTime(unsigned index) const;

    /**
     * @return the start time of the composite behavior description
     */
    unsigned long getStartTime() const;


    /**
     * @return the end time of the composite behavior description
     */
    unsigned long getEndTime() const;

    /**
     * @return the time interval of the composite behavior description
     */
    Temporal getTimeInterval() const;

    /**
     * @return A reference for the entries vector
     */
    const std::vector<ElementaryBehaviorDescription> &getEntries() const;

    /**
     * return true iff the CompositeBehaviorDescription is empty()
     */
    bool empty() const;

    /**
     * return the number of elementary behvaior description
     */
    unsigned int size() const;

    /**
     * Remove all elements from BD. If none, does nothing.
     */
    void clear();

    /**
     * This method is used in BehaviorCategory to build timeline representation of a set of
     * behavior descriptions. It is not exactly part of CompositeBehaviorDescription API.
     */
    static std::vector<unsigned long> *buildSeparatorsVector(const std::vector<ElementaryBehaviorDescription> &entries);

    std::string toString() const;

    // ***********************************************/
    // Test/debug

    bool equals(const CompositeBehaviorDescription &other) const;
    std::string toStringHandles();
    std::string toStringTimeline();
    std::string toStringTimeline(std::vector<PredicateHandleSet> &timelineSets, std::vector<long> &timelineIntervals);


private:

    AtomSpace* atomspace;

    mutable std::vector<ElementaryBehaviorDescription> entries;
    mutable std::vector<PredicateHandleSet> timelineSets;
    mutable std::vector<long> timelineIntervals;


    /**
     * Timeline representation is used by BehaviorDescriptionMatcher. This flag is used
     * to allow lazzy building of such a representation when the user call the appropriate
     * methods to query for timeline contents.
     */
    mutable bool timelineRepresentationIsValid;

    /**
     * See comments on variable timelineRepresentationIsValid above
     */
    void buildTimelineRepresentation() const;
    void buildTimelineRepresentation(std::vector<PredicateHandleSet> &timelineSets, std::vector<long> &timelineIntervals, std::vector<ElementaryBehaviorDescription> &entries) const;
}; // class
}  // namespace

#endif
