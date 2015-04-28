/*
 * opencog/embodiment/Learning/NoSpaceLife/NoSpaceLife.cc
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

#include <opencog/util/random.h>

#include <opencog/atomspace/Node.h>

#include <opencog/embodiment/AvatarComboVocabulary/avatar_builtin_action.h>
#include <opencog/embodiment/AvatarComboVocabulary/avatar_perception.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/WorldWrapper/WorldWrapperUtil.h>

#include "NoSpaceLife.h"

namespace ImaginaryLife
{

using namespace opencog::world;
using namespace AvatarCombo;

/**
 * public methods
 */

NoSpaceLife::NoSpaceLife(AtomSpace& atomSpace, const std::string& pet_id,
                         const std::string& owner_id,
                         const std::string& avatar_id,
                         const CompositeBehaviorDescription& cbd,
                         const Temporal& et) :
        _atomSpace(atomSpace), _pet_id(pet_id), _owner_id(owner_id),
        _avatar_id(avatar_id),
        _currentTime(0), _currentIndex(0), _currentMapHandle(Handle::UNDEFINED),
        _hasTimeChanged(true), _imitatedBD(cbd), _exemplarTemporal(et),
        _generatedBD(&atomSpace)
{
    _currentTime = _imitatedBD.getStartTime();
}
NoSpaceLife::~NoSpaceLife() {}

bool NoSpaceLife::processSequential_and(sib_it from, sib_it to)
{
    unsigned long end_time = 0;
    const std::vector<long>& tl = _imitatedBD.getTimelineIntervals();
    while (from != to) {
        OC_ASSERT(is_builtin_action(*from));
        //determine action start and end time
        unsigned long start_time = _currentTime;
        if (_currentIndex < tl.size())
            end_time = start_time + tl[_currentIndex];
        else end_time = start_time + ACTION_DONE_TIME;
        //add action
        ElementaryBehaviorDescription ebd;
        generateElementaryBD(ebd, pre_it(from), start_time, end_time);
        _generatedBD.addPredicate(ebd);
        //determine pause end time
        _currentIndex++;
        if (_currentIndex < tl.size())
            end_time += tl[_currentIndex];
        else if (_currentIndex == tl.size())
            end_time = _exemplarTemporal.getUpperBound();
        else end_time = end_time + PAUSE_TIME;

        //debug print
        //std::cout << "CURRENT INDEX : " << _currentIndex
        // << " TL SIZE : " << tl.size()
        // << " CURRENT TIME : " << _currentTime
        // << " IMITATED BD END TIME : " << _imitatedBD.getEndTime()
        // << std::endl;
        //~debug print

        _currentTime = end_time;
        ++from;
        _currentIndex++;
    }
    return true;
}

CompositeBehaviorDescription& NoSpaceLife::getGeneratedBD()
{
    return _generatedBD;
}

Handle NoSpaceLife::getCurrentMapHandle()
{
    if (_hasTimeChanged) {
        _currentMapHandle
        = AtomSpaceUtil::getCurrentSpaceMapHandle(_atomSpace);
        _hasTimeChanged = false;
    }
    return _currentMapHandle;
}

unsigned long NoSpaceLife::getCurrentTime() const
{
    return _currentTime;
}

AtomSpace& NoSpaceLife::getAtomSpace() const
{
    return _atomSpace;
}

/**
 * private methods
 */

void NoSpaceLife::generateElementaryBD(ElementaryBehaviorDescription& ebd,
                                       pre_it it,
                                       unsigned long start_time,
                                       unsigned long end_time)
{
    OC_ASSERT(is_builtin_action(*it));

    builtin_action a = get_builtin_action(*it);
    HandleSeq hs;
    Handle h;

    //add the subject of the action to create
    hs.push_back(_atomSpace.addNode(PET_NODE, _pet_id));

    //add the action
    if (a == get_instance(id::random_step))
        a = choose_random_step();
    std::stringstream ss;
    ss << a;
    hs.push_back(_atomSpace.addNode(NODE, ss.str()));

    //add the arguments
    int arg_index = 0;
    for (sib_it sib = it.begin(); sib != it.end(); ++sib, arg_index++) {
        //argument to add to action of the elementary behavior description
        Handle arg_h = Handle::UNDEFINED;
        //definite or indefinite object case
        if (is_definite_object(*sib) || is_indefinite_object(*sib)) {
            definite_object obj;
            if (is_definite_object(*sib))
                obj = get_definite_object(*sib);
            else if (is_indefinite_object(*sib)) {
                indefinite_object io = get_indefinite_object(*sib);
                OC_ASSERT(is_random(io),
                                 "%s must be random",
                                 io->get_name().c_str());
                //determine the definite object obj that fits the best
                //the behavior description at the particular moment
                obj = choose_definite_object_that_fits(io, arg_index);
            }
            arg_h = WorldWrapperUtil::toHandle(_atomSpace, obj, _pet_id, _owner_id);
        }
        //contin case
        else if (is_contin(*sib)) {
            arg_h = _atomSpace.addNode(NUMBER_NODE,
                                       boost::lexical_cast<std::string>(get_contin(*sib)));
        } else {
            OC_ASSERT(false,
                             "That case is not handled yet");
        }
        OC_ASSERT(arg_h != Handle::UNDEFINED);
        hs.push_back(arg_h);
    }

    //Create the output of the EvalLink
    HandleSeq eo;
    //add predicatNode:"behaved"
    h = _atomSpace.addNode(PREDICATE_NODE, BEHAVED_STR);
    eo.push_back(h);
    //create and add listLink with hs to eo
    h = _atomSpace.addLink(LIST_LINK, hs);
    eo.push_back(h);
    //create the EvaluationLink
    h = _atomSpace.addLink(EVALUATION_LINK, eo);
    //fill the elementary BD
    Temporal t(start_time, end_time);
    ebd.handle = h;
    ebd.temporal = t;
}

builtin_action NoSpaceLife::choose_random_step() const
{
    int c = randGen().randint(4);
    switch (c) {
    case 0:
        return get_instance(id::step_backward);
    case 1:
        return get_instance(id::step_forward);
    case 2:
        return get_instance(id::rotate_left);
    case 3:
        return get_instance(id::rotate_right);
    default:
        OC_ASSERT(false,
                         "Combo2BD::choose_random_step() : UNKNOWN CASE");
        return get_instance(id::random_step);
    }
}

definite_object NoSpaceLife::choose_definite_object_that_fits(indefinite_object io, int arg_index)
{
    //get the elementary BD at current time
    const std::vector<PredicateHandleSet>& tls = _imitatedBD.getTimelineSets();

    bool corresponding_arg_exists = false;
    std::string do_id; //id (or name) of the definite_object to return;

    if (_currentIndex < tls.size()) {
        const PredicateHandleSet& hs = tls[_currentIndex];
        unsigned s = hs.getSize();
        OC_ASSERT(s == 0 || s == 1, "hs should have at 0 or 1 element.");
        if (s > 0) { //if there is an element then check if it could fit

            //get the Handle of the arg_index argument if there is
            Handle h = *hs.getSet().begin();
            //check that it matches Behavior Description atom structure
            OC_ASSERT(h != Handle::UNDEFINED);
            OC_ASSERT(_atomSpace.getType(h) == EVALUATION_LINK);
            OC_ASSERT(_atomSpace.getArity(h) == 2, "An EvaluationLink must have only 2 arguments");

            Handle list_h = _atomSpace.getOutgoing(h, 1);
            OC_ASSERT(_atomSpace.getType(list_h) == LIST_LINK);
            //check if the equivalent argument exists
            //and can correspond to a random operator
            if (arg_index < _atomSpace.getArity(list_h) - 2) {
                Handle arg_h = _atomSpace.getOutgoing(list_h, arg_index + 2);
                OC_ASSERT(arg_h != Handle::UNDEFINED);
                OC_ASSERT(_atomSpace.getType(arg_h),
                                 "arg_h must be a Node");
                //convert to self or owner if name is _avatar_id or _owner_id
                //(avatarName corresponds to self because the puts itself under its
                //skin
                do_id = WorldWrapperUtil::atom_name_to_definite_object(_atomSpace.getName(arg_h), _avatar_id, _owner_id);
                //depending on the random operator check if the object belongs to
                //the set
                if (io == get_instance(id::random_object))
                    corresponding_arg_exists = true;
                else {
                    perception p = WorldWrapperUtil::nearest_random_X_to_is_X(io);
                    pre_it it = WorldWrapperUtil::maketree_percept(p, do_id);
                    corresponding_arg_exists =
                        combo::vertex_to_bool(
                            WorldWrapperUtil::evalPerception(getCurrentMapHandle(),
                                                             _currentTime, _atomSpace,
                                                             _avatar_id, _owner_id,
                                                             it, true));
                }
            }
        }
    }
    //if a corresponding argument exist then take it
    //otherwise choose one randomly according to the random operator io
    if (corresponding_arg_exists) {
        return definite_object(do_id);
    } else {
        vertex v = WorldWrapperUtil::evalIndefiniteObject(
                   getCurrentMapHandle(),
                   _currentTime,
                   _atomSpace,
                   _avatar_id,
                   _owner_id,
                   io,
                   true);
        OC_ASSERT(is_definite_object(v));
        return get_definite_object(v);
    }
}

}
