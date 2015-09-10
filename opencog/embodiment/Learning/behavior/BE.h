/*
 * opencog/embodiment/Learning/behavior/BE.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef BEHAVIOR_BE_H
#define BEHAVIOR_BE_H

#include <algorithm>

#include <opencog/spacetime/Temporal.h>
#include <opencog/embodiment/AtomSpaceExtensions/CompareAtomTreeTemplate.h>

#include "BDTracker.h"
#include "WorldProvider.h"
#include "CompositeBehaviorDescription.h"

#define BEHAVED_PREDICATE_NAME "behaved"

using namespace opencog;

namespace behavior
{

//class BDTracker;

// For each BD type, we create 1. a predicate that identifies valid input (perception) Atoms
// that provide info for the BD, 2. a BDTracker which keeps track of all objects that have
// been observed in perceptions of that kind, and ObjectTracker which uses a new percept
// to update the corresponding BD of the object

#include <opencog/util/tree.h>

struct less_combo_tree : public std::binary_function<tree<Vertex>, tree<Vertex>, bool> {
    bool operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs) const;
};

class BehaviorEncoder : public BDCreationListener
{
protected:
    Handle trickExemplarAtTime; // the handle of the AtTimeLink for the exemplar of the trick being learned.
    int time_resolution;
    Temporal next_moment;


    std::map< tree<Vertex>, BDTracker*, less_combo_tree> factories;
    typedef std::pair< tree<Vertex>, BDTracker*> VFpair;

    std::set<Handle> new_bds;
    WorldProvider* wp;
    const std::string& pet_id; //this is used by tempUpdateRec so that it does not track the pet itself which is observing and not participating to the trick
public:

    /// Keep a record of all BDs created by my BD factories.
    void onBD(Handle bd, Handle timed_bd);

    /// time_resolution is the minimum time that needs to lapse before
    /// we update again
    BehaviorEncoder(WorldProvider* _wp, const std::string& _pet_id, Handle _trickExemplarAtTime, int _time_resolution, Temporal _next_moment = Temporal(0));
    /// Add a behavior encoding rule that takes in atoms that satisfy the predicate.
    void addBETracker(const tree<Vertex>& atom_template, BDTracker* tracker);

    ~BehaviorEncoder();

    std::set<Handle>::const_iterator NewBDs_begin() {
        return new_bds.begin();
    }
    std::set<Handle>::const_iterator NewBDs_end() {
        return new_bds.end();
    }
    unsigned int NewBDs_size() {
        return new_bds.size();
    }
    void ClearNewBDs() {
        new_bds.clear();
    }

    /// Update the repository with Atoms that have emerged since start_moment.
    /// If start_moment is not provided, the method will use start_moment=next_moment.
    /// If start_moment <= next_moment, update next_moment to NOW
    /// You can skip some atoms while keeping the next_moment pointer valid
    /// by using SetNextMoment().
    /// Returns true if now > next_moment+time_resolution .
    /// Else, doesn't update and returns false.
    bool update(Temporal start_moment = Temporal(0));

    //perform a loop of updates from the beginning of the exemplar until the end
    void updateRec(Temporal exemplarInterval);

    //Quick recoding of the update method to work in non-real-time
    void tempUpdateRec(Temporal exemplarInterval);

    bool isUpdated() const;

    void SetNextMoment(const Temporal& _next_moment) {
        next_moment = _next_moment;
    }

// We may want to support these later, or not:

    /// Add callbacks for the event of a new behavior having been encoded.
    //void addBEListener(boost::shared_ptr<BEListener>);
    //bool update(Atom* a);

    static const unsigned long MinActionTime;
    static const unsigned long MinPauseTime;

};

} //namespace behavior

#endif
