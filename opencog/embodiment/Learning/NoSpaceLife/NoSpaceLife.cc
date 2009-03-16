/**
 * NoSpaceLife.cc
 *
 * Author(s):
 *   Nil Geisweiller
 * Creation: Wed Dec 5 2007
 */
#include "NoSpaceLife.h"
#include "AtomSpaceUtil.h"
#include "WorldWrapperUtil.h"
#include "util/RandGen.h"
#include <opencog/atomspace/Node.h>
#include "pet_builtin_action.h"
#include "pet_perception.h"

namespace ImaginaryLife
{

using namespace WorldWrapper;
using namespace PetCombo;

/**
 * public methods
 */

NoSpaceLife::NoSpaceLife(SpaceServer& spaceServer, const std::string& pet_id,
                         const std::string& owner_id,
                         const std::string& avatar_id,
                         const CompositeBehaviorDescription& cbd,
                         const Temporal& et,
                         opencog::RandGen& rng) :
    _spaceServer(spaceServer), _pet_id(pet_id), _owner_id(owner_id),
    _avatar_id(avatar_id),
    _currentTime(0), _currentIndex(0), _currentMapHandle(Handle::UNDEFINED),
    _hasTimeChanged(true), _imitatedBD(cbd), _exemplarTemporal(et), _rng(rng)
{
    _currentTime = _imitatedBD.getStartTime();
}
NoSpaceLife::~NoSpaceLife() {}

bool NoSpaceLife::processSequential_and(sib_it from, sib_it to)
{
    unsigned long end_time = 0;
    const std::vector<long>& tl = _imitatedBD.getTimelineIntervals();
    while (from != to) {
        opencog::cassert(TRACE_INFO, is_builtin_action(*from));
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
        = AtomSpaceUtil::getSpaceMapHandleAtTimestamp(_spaceServer,
                _currentTime);
        _hasTimeChanged = false;
    }
    return _currentMapHandle;
}

unsigned long NoSpaceLife::getCurrentTime()
{
    return _currentTime;
}

AtomSpace& NoSpaceLife::getAtomSpace() const
{
    return _spaceServer.getAtomSpace();
}

/**
 * private methods
 */

void NoSpaceLife::generateElementaryBD(ElementaryBehaviorDescription& ebd,
                                       pre_it it,
                                       unsigned long start_time,
                                       unsigned long end_time)
{
    opencog::cassert(TRACE_INFO, is_builtin_action(*it));

    AtomSpace& as = _spaceServer.getAtomSpace();

    builtin_action a = get_builtin_action(*it);
    HandleSeq hs;
    Handle h;

    //add the subject of the action to create
    hs.push_back(as.addNode(SL_PET_NODE, _pet_id));

    //add the action
    if (a == instance(id::random_step))
        a = choose_random_step();
    std::stringstream ss;
    ss << a;
    hs.push_back(as.addNode(NODE, ss.str()));

    //add the arguments
    int arg_index = 0;
    for (sib_it sib = it.begin(); sib != it.end(); ++sib, arg_index++) {
        //argument to add to action of the elementary behavior description
        Handle arg_h = Handle::UNDEFINED;
        //definite or indefinite object case
        if(is_definite_object(*sib) || is_indefinite_object(*sib)) {
            definite_object obj;
            if (is_definite_object(*sib))
                obj = get_definite_object(*sib);
            else if (is_indefinite_object(*sib)) {
                indefinite_object io = get_indefinite_object(*sib);
                opencog::cassert(TRACE_INFO, is_random(io),
                                  "%s must be random",
                                  io->get_name().c_str());
                //determine the definite object obj that fits the best
                //the behavior description at the particular moment
                obj = choose_definite_object_that_fits(io, arg_index);
            }
            arg_h = WorldWrapperUtil::toHandle(as, obj, _pet_id, _owner_id);
        }
        //contin case
        else if (is_contin(*sib)) {
            arg_h = as.addNode(NUMBER_NODE,
                               boost::lexical_cast<std::string>(get_contin(*sib)));
        }
        else {
            opencog::cassert(TRACE_INFO, false,
                              "That case is not handled yet");
        }
        opencog::cassert(TRACE_INFO, arg_h != Handle::UNDEFINED);
        hs.push_back(arg_h);
    }                 

    //Create the output of the EvalLink
    HandleSeq eo;
    //add predicatNode:"behaved"
    h = as.addNode(PREDICATE_NODE, BEHAVED_STR);
    eo.push_back(h);
    //create and add listLink with hs to eo
    h = as.addLink(LIST_LINK, hs);
    eo.push_back(h);
    //create the EvaluationLink
    h = as.addLink(EVALUATION_LINK, eo);
    //fill the elementary BD
    Temporal t(start_time, end_time);
    ebd.handle = h;
    ebd.temporal = t;
}

builtin_action NoSpaceLife::choose_random_step() const
{
    int c = _rng.randint(4);
    switch (c) {
    case 0:
        return instance(id::step_backward);
    case 1:
        return instance(id::step_forward);
    case 2:
        return instance(id::rotate_left);
    case 3:
        return instance(id::rotate_right);
    default:
        opencog::cassert(TRACE_INFO, false,
                          "Combo2BD::choose_random_step() : UNKNOWN CASE");
        return instance(id::random_step);
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
        opencog::cassert(TRACE_INFO, s == 0 || s == 1, "hs should have at 0 or 1 element.");
        if (s > 0) { //if there is an element then check if it could fit

            //get the Handle of the arg_index argument if there is
            Handle h = *hs.getSet().begin();
            //check that it matches Behavior Description atom structure
            opencog::cassert(TRACE_INFO, h != Handle::UNDEFINED);
            opencog::cassert(TRACE_INFO, _spaceServer.getAtomSpace().getType(h) == EVALUATION_LINK);
            opencog::cassert(TRACE_INFO, _spaceServer.getAtomSpace().getArity(h) == 2, "An EvaluationLink must have only 2 arguments");

            Handle list_h = _spaceServer.getAtomSpace().getOutgoing(h, 1);
            opencog::cassert(TRACE_INFO, _spaceServer.getAtomSpace().getType(list_h) == LIST_LINK);
            //check if the equivalent argument exists
            //and can correspond to a random operator
            if (arg_index < _spaceServer.getAtomSpace().getArity(list_h) - 2) {
                Handle arg_h = _spaceServer.getAtomSpace().getOutgoing(list_h, arg_index + 2);
                opencog::cassert(TRACE_INFO, arg_h != Handle::UNDEFINED);
                opencog::cassert(TRACE_INFO, dynamic_cast<Node*>(TLB::getAtom(arg_h)),
                                  "arg_h must be a Node");
                //convert to self or owner if name is _avatar_id or _owner_id
                //(avatarName corresponds to self because the puts itself under its
                //skin
                do_id = WorldWrapperUtil::atom_name_to_definite_object(_spaceServer.getAtomSpace().getName(arg_h), _avatar_id, _owner_id);
                //depending on the random operator check if the object belongs to
                //the set
                if (io == instance(id::random_object))
                    corresponding_arg_exists = true;
                else {
                    perception p = WorldWrapperUtil::nearest_random_X_to_is_X(io);
                    pre_it it = WorldWrapperUtil::maketree_percept(p, do_id);
                    corresponding_arg_exists =
                        combo::vertex_to_bool(
                            WorldWrapperUtil::evalPerception(_rng, getCurrentMapHandle(),
                                                             _currentTime, _spaceServer,
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
        vertex v = WorldWrapperUtil::evalIndefiniteObject(_rng,
                   getCurrentMapHandle(),
                   _currentTime,
                   _spaceServer,
                   _avatar_id,
                   _owner_id,
                   io,
                   true);
        opencog::cassert(TRACE_INFO, is_definite_object(v));
        return get_definite_object(v);
    }
}

}
