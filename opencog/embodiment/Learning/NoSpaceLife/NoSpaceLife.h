/*
 * opencog/embodiment/Learning/NoSpaceLife/NoSpaceLife.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _NOSPACELIFE_H
#define _NOSPACELIFE_H

#include <opencog/comboreduct/combo/vertex.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/embodiment/Learning/behavior/CompositeBehaviorDescription.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#include <opencog/util/RandGen.h>

#define ACTION_DONE_TIME 200 //each action takes 2 seconds
#define PAUSE_TIME 200
#define BEHAVED_STR "behaved"

namespace ImaginaryLife
{

using namespace behavior;
using namespace combo;

class NoSpaceLife
{
public:
    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;

    /**
     * cbd is the CompositeBehaviorDescription to imitate
     * the reason it is provided is because the generated BD will uses
     * the timestamps of cbd to calculate the action durations
     * and pause durations
     */
    NoSpaceLife(AtomSpace& atomSpace, const std::string& pet_id,
                const std::string& owner_id, const std::string& avatar_id,
                const CompositeBehaviorDescription& cbd,
                const Temporal& exemplarTemporal,
                opencog::RandGen& rng);

    ~NoSpaceLife();

    /**
     * process a list of actions [from, to),
     * is it assumed that all objects are definite
     * returns true iff the action sequence has been successful
     */
    bool processSequential_and(sib_it from, sib_it to);

    /**
     * get the generated BD
     */
    CompositeBehaviorDescription& getGeneratedBD();

    /**
     * get the current SpaceMap
     */
    Handle getCurrentMapHandle();

    unsigned long getCurrentTime() const;

    AtomSpace& getAtomSpace() const;

private:
    AtomSpace& _atomSpace; //a reference to the AtomSpace that lives within LS
    //not a const because it can be changed by the algo

    const std::string& _pet_id;
    const std::string& _owner_id;
    const std::string& _avatar_id; //the avatar to imitate
    //const definite_object_set& _definite_objects; //reference to
    //definite_objects
    //that leaves within HCTask

    unsigned long _currentTime;

    unsigned _currentIndex;

    Handle _currentMapHandle;

    bool _hasTimeChanged;

    const CompositeBehaviorDescription& _imitatedBD;

    const Temporal& _exemplarTemporal;

    CompositeBehaviorDescription _generatedBD;

    opencog::RandGen& _rng;

    void generateElementaryBD(ElementaryBehaviorDescription& ebd,
                              pre_it it,
                              unsigned long start_time,
                              unsigned long end_time);

    builtin_action choose_random_step() const;

    definite_object choose_definite_object_that_fits(indefinite_object obj,
            int arg_index);

};

}//~namespace ImaginaryLife

#endif
