/*
 * opencog/embodiment/Learning/behavior/BE.cc
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

typedef unsigned long ulong;
#include <stdio.h>
#include <map>
#include <set>

#include <opencog/util/functional.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/misc.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spacetime/TimeServer.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/AtomSpaceExtensions/CompareAtomTreeTemplate.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#include "BE.h"
#include "BDTracker.h"
#include "WorldProvider.h"

#define WALK_PERCEPT_NAME "walk"
#define MIN_ACTION_DURATION 2
#define MIN_PAUSE_DURATION 2
#define GOTO_OBJ_SPEED 2.5

#if defined( _WIN32) && !defined(__CYGWIN__)
typedef __int64 mytime_t;
#else
#include <inttypes.h> // int64_t
typedef int64_t mytime_t;
#endif

namespace behavior
{

using namespace AvatarCombo;
using namespace opencog;

const unsigned long BehaviorEncoder::MinActionTime = MIN_ACTION_DURATION;
const unsigned long BehaviorEncoder::MinPauseTime = MIN_PAUSE_DURATION;

bool behlog = false;
#define bprintf(__barg1, __barg2) if (behlog) printf(__barg1, __barg2)

class ExcludeObjsIdPred
{
private:
    std::set<std::string> objIdSet;
public:
    ExcludeObjsIdPred(const std::string& _objId) {
        objIdSet.insert(_objId);
    }
    ExcludeObjsIdPred(const std::string& _objId1, const std::string& _objId2) {
        objIdSet.insert(_objId1);
        objIdSet.insert(_objId2);
    }
    void insert(const std::string& _objId) {
        objIdSet.insert(_objId);
    }
    bool operator()(const std::string& _objId) const {
        return objIdSet.find(_objId) == objIdSet.end();
    }
};

bool less_combo_tree_it( const tree<Vertex> & lhs_t, const tree<Vertex> & rhs_t,
                         tree<Vertex>::sibling_iterator ltop, tree<Vertex>::sibling_iterator rtop)
{
    if ((*ltop) < (*rtop))
        return true;
    if ((*rtop) < (*ltop))
        return false;

    tree<Vertex>::sibling_iterator rit = rhs_t.begin(rtop);

    for (tree<Vertex>::sibling_iterator lit = lhs_t.begin(ltop);
            lit != lhs_t.end  (ltop);
            ++lit, ++rit)
        if (less_combo_tree_it(lhs_t, rhs_t, lit, rit))
            return true;
        else if (less_combo_tree_it(lhs_t, rhs_t, rit, lit))
            return false;

    return false;
}

bool less_combo_tree::operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs) const
{
    if (lhs.size() < rhs.size())
        return true;
    if (lhs.size() > rhs.size() || lhs.empty())
        return false;

    return less_combo_tree_it(lhs, rhs, lhs.begin(), rhs.begin());
}

BehaviorEncoder::BehaviorEncoder(WorldProvider* _wp, const string& _pet_id, Handle _trickExemplarAtTime, int _time_resolution, Temporal _next_moment)
        : trickExemplarAtTime(_trickExemplarAtTime), time_resolution(_time_resolution), next_moment(_next_moment), wp(_wp), pet_id(_pet_id) {}


void BehaviorEncoder::addBETracker(const tree<Vertex>& atom_template,
                                   BDTracker* tracker)
{
    factories[atom_template] = tracker;
    tracker->addBDCreationListener(this);
}

void BehaviorEncoder::onBD(Handle bd, Handle timed_bd)
{
// printf("New bd created. (size was %d\n", new_bds.size());
    new_bds.insert(bd);
    /*
    // printf("%d\n", (int)*NewBDs_begin());
     puts((*NewBDs_begin()++)->toString().c_str());
     puts("Integrity test...");

     std::set<Handle>::const_iterator it = NewBDs_begin();
     for (int b=0; b < NewBDs_size(); b++)
      strlen(((*it++)->toString().c_str()));
    */
}

/**
 * recoded a simpler non-real-time BDTracker
 * It takes an action_done predicate of the following form
 *
 * EvalLink
 *     actionDone
 *     ListLink
 *         subject
 *         action
 *         ListLink
 *             arg1
 *             ...
 *
 * into
 *
 * EvalLink
 *     behaved
 *     ListLink
 *         subject
 *         action
 *         arg1
 *         ...
 *
 * So the arguments of the subject's action are flattened and also
 * goto_obj are generated out of walk commands
 */
void BehaviorEncoder::tempUpdateRec(Temporal exemplarInterval)
{/*
    double dist_ratio_threshold = opencog::config().get_double("DIST_PERCENTAGE_THRESHOLD_WALK_TO_GOTO") / (double)100;

    AtomSpace& as = wp->getAtomSpace();

    foreach(const VFpair& vf, factories) {
        vector<HandleTemporalPair> temps;
        as.getTimeServer().getTimeInfo(inserter(temps, temps.begin()),
                       Handle::UNDEFINED,
                       exemplarInterval,
                       TemporalTable::STARTS_WITHIN);
        unsigned long tl = 0; //perception time lower bound
        unsigned long tu = 0; //perception time upper bound
        unsigned long previous_tu = 0; //end time of the previous action
        for (vector<HandleTemporalPair>::iterator htp_i = temps.begin();
                htp_i != temps.end(); ++htp_i) {
            Handle h = htp_i->getHandle();
            does_fit_template does_match_template =
                does_fit_template(vf.first, &as);
            if (does_match_template(h)) {
                logger().debug("BE - The following atom structure has been selected to be an elementary behavior description or contributing to define one: %s", as.atomAsString(h).c_str());

                //create BD
                OC_ASSERT(as.getType(h) == EVALUATION_LINK,
                                 "Handle h should be an 'EVALUATION_LINK'.");
                OC_ASSERT(as.getArity(h) == 2,
                                 "Handle h should have arity 2.");

                Handle behaved_h = as.addNode(PREDICATE_NODE, BEHAVED_PREDICATE_NAME);

                Handle arg_list_h = as.getOutgoing(h, 1);
                OC_ASSERT(as.getType(arg_list_h) == LIST_LINK,
                                 "Handle h outgoingAtom[1] should be a 'LIST_LINK'.");

                //determine the subject
                Handle subject_h = as.getOutgoing(arg_list_h, 0);
                const std::string& subject_id = as.getName(subject_h);

                //to be checked if walk perceptions
                //per_h is in fact the action of the subject observed
                Handle per_h = as.getOutgoing(arg_list_h, 1);
                OC_ASSERT(as.isNode(per_h), "Handle per_h should be a 'Node'.");

                bool is_walk = as.getName(per_h) == WALK_PERCEPT_NAME;

                //deternime tl and tu of the perception

                //if tl==0 then it is assumed
                //that the first walk of the goto_obj is being determined
                if (!is_walk || tl == 0) {
                    tl = htp_i->getTemporal()->getLowerBound();
                }

                tu = htp_i->getTemporal()->getUpperBound();
                //put that hack to not have instantaneous actions
                if (tl + MinActionTime >= tu)
                    tu = tl + MinActionTime;

                //contain a list of (subject, action, arg1, ..., argn)
                //a new handle is needed because the actionDone predicate
                //has its arguments embeded into a list
                Handle new_arg_list_h;

                if (is_walk) {

                    //get walk destination coordinates
                    //check if the walk destination is close to an object

                    OC_ASSERT(as.getArity(arg_list_h) > 2,
                                     "arg_list_h should have more than 2 outgoings" );

                    Handle actionParametersList_h = as.getOutgoing(arg_list_h, 2);

                    //get the position of the walk destination
                    OC_ASSERT(as.getArity(actionParametersList_h) > 0,
                                     "ListLink arg_list should have arity greater than 0 (walk perception).");

                    Handle pos_list_h = as.getOutgoing(actionParametersList_h, 0);
                    OC_ASSERT(as.getType(pos_list_h) == LIST_LINK,
                                     "Handle pos_list should be a 'LIST_LINK' (walk perception)");
                    OC_ASSERT(as.getArity(pos_list_h) == 3,
                                     "ListLink pos_list should have arity 3 (walk perception)." );

                    Handle x_h = as.getOutgoing(pos_list_h, 0);
                    OC_ASSERT(as.getType(x_h) == NUMBER_NODE,
                                     "Handle x_h should be a 'NUMBER_NODE'.");
                    float x = atof(as.getName(x_h).c_str());
                    Handle y_h = as.getOutgoing(pos_list_h, 1);
                    OC_ASSERT(as.getType(y_h) == NUMBER_NODE,
                                     "Handle y_h should be a 'NUMBER_NODE'.");
                    float y = atof(as.getName(y_h).c_str());
                    SpaceServer::SpaceMapPoint p_walk_dest(x, y);

                    // define pred to find nearest obj
                    //except for the subject itself and the observer pet
                    ExcludeObjsIdPred pred(subject_id, pet_id);

                    //expect for the holded object (if such)
                    //it is assumed that if the object is holded at the start of
                    //the walk command then it is still holded at the end of the
                    //walk command
                    std::string held_object_id =
                        AtomSpaceUtil::getHoldingObjectIdAtTime(as,
                                                                subject_id, tu);
                    bool held_object = held_object_id != "";
                    if (held_object) {
                        pred.insert(held_object_id);
                    }

                    //LOGGER DEBUG
                    if (held_object)
                        logger().debug("BE - Detected a walk command with the time interval (%u, %u), from the agent %s holding the object %s", tl, tu, subject_id.c_str(), held_object_id.c_str());
                    else
                        logger().debug("BE - Detected a walk command with the time interval (%u, %u), from the agent %s holding no object", tl, tu, subject_id.c_str());
                    //~LOGGER DEBUG

                    //and except the nearest object (if such) at the start of
                    //the walk command

                    //get the position of the avatar at the start of the walk
                    //COULD BE OPTIMIZED
                    Handle sms_h = AtomSpaceUtil::getSpaceMapHandleAtTimestamp(wp->getAtomSpace(), tl);
                    OC_ASSERT(sms_h != Handle::UNDEFINED,
                                     "Handle sms_h should not be an 'Handle::UNDEFINED'.");
                    const SpaceServer::SpaceMap& sms = wp->getAtomSpace().getSpaceServer().getMap(sms_h);
                    //sas stands for Subject At Start
                    const spatial::EntityPtr& sasEntity = sms.getEntity(subject_id);

                    //get the position of the nearest object to the subject at the
                    //start of the walk command
                    std::string obj_id_nearest_sas =
                        sms.findNearestFiltered(spatial::Point(sasEntity->getPosition( ).x,
                                                               sasEntity->getPosition( ).y), pred);

                    const spatial::EntityPtr& nearestSasEntity = sms.getEntity(obj_id_nearest_sas);

                    //check that they are close enough
                    //if so then add that object into the set of objects that cannot be
                    //destination of goto_obj
                    double dist_s = (sasEntity->getPosition()
                                     - nearestSasEntity->getPosition()).length();

                    if (dist_s <= dist_ratio_threshold*sms.diagonalSize()) {
                        logger().debug("BE - the nearest object %s at the start of walk command is at distance %f from the agent which is too close to be in the list of potential objects of destination", obj_id_nearest_sas.c_str(), dist_s);

                        pred.insert(obj_id_nearest_sas);
                    } else {
                        logger().debug("BE - the nearest object %s at the start of walk command is at distance %f from the agent which is far enough to be in the list of potential objects of destination", obj_id_nearest_sas.c_str(), dist_s);
                    }

                    //find the nearest object to the subject at the end of the walk
                    //which is different of the object at the start if there is
                    Handle sm_h = AtomSpaceUtil::getSpaceMapHandleAtTimestamp(wp->getAtomSpace(), tu);
                    OC_ASSERT(sm_h != Handle::UNDEFINED,
                                     "Handle sm_h should not be an 'Handle::UNDEFINED'.");
                    const SpaceServer::SpaceMap& sm = wp->getAtomSpace().getSpaceServer().getMap(sm_h);
                    std::string obj_id = sm.findNearestFiltered(p_walk_dest, pred);

                    const spatial::EntityPtr& objectEntity = sm.getEntity(obj_id);

                    const spatial::EntityPtr& subjectEndEntity = sm.getEntity(subject_id);

                    double dist_subject_end = objectEntity->distanceTo(*subjectEndEntity);


                    //WARNING : not used because position after a walk command and of walk command destination does not coincide
                    //double dist = (objectEntity->getPosition() - spatial::math::Vector3(p.first, p.second)).length();
                    //SpaceServer::SpaceMap::eucDist(p, obj_p);
                    //debug print
                    //std::cout << "NEAREST OBJECT AT THE END OF THE WALK COMMAND ACCORDING WALK DESTINATION : "
                    //    << obj_id
                    //        << objectEntity->getPosition().toString()
                    //<< " X : " << md.centerX
                    //<< " Y : " << md.centerY
                    //    << " DIST : " << dist << std::endl;
                    //~debug print

                    //debug print
                    //std::cout << "NEAREST OBJECT AT THE END OF THE WALK COMMAND : "
                    //    << obj_id << " DIST : " << dist_subject_end
                    //    << " DIST THRESHOLD : "
                    //    << dist_ratio_threshold*sm.diagonalSize() << std::endl;
                    //~debug print

                    //debug print
                    //Handle next_sm_h =  wp->getAtomSpace().getSpaceServer().getNextMapHandle(sm_h);
                    //if(next_sm_h!=Handle::UNDEFINED) {
                    //const SpaceServer::SpaceMap& next_sm = wp->getAtomSpace().getSpaceServer().getMap(next_sm_h);

                    //const spatial::EntityPtr& nextObjectEntity = sm.getEntity(obj_id);
                    //const spatial::EntityPtr& nextSubjectEndEntity = next_sm.getEntity(subject_id);
                    //double next_dist = (nextSubjectEndEntity->getPosition() - nextObjectEntity->getPosition()).length();
                    //std::cout << "NEAREST OBJECT AT THE NEXT MAP : " << obj_id
                    //      << " DIST : " << next_dist << std::endl;
                    //}
                    //else {
                    //std::cout << "NEXT_SM_H IS Handle::UNDEFINED" << std::endl;
                    //}
                    //~debug print

                    //debug print
                    //Handle instant_next_sm_h = AtomSpaceUtil::getSpaceMapHandleAtTimestamp(wp->getAtomSpace(), tu+1);
                    //OC_ASSERT(instant_next_sm_h!=Handle::UNDEFINED,
                    //     "Handle sm_h should not be an 'Handle::UNDEFINED'.");
                    //const SpaceServer::SpaceMap& instant_next_sm = wp->getAtomSpace().getSpaceServer().getMap(instant_next_sm_h);

                    //const SpaceServer::ObjectMetadata& instant_next_md_obj = instant_next_sm.getMetaData(obj_id);
                    //SpaceServer::SpaceMapPoint instant_next_obj_p(instant_next_md_obj.centerX, instant_next_md_obj.centerY);
                    //const spatial::EntityPtr& instantNextObjectEntity = instant_next_sm.getEntity(obj_id);

                    //const SpaceServer::ObjectMetadata& instant_next_md_subject_end = instant_next_sm.getMetaData(subject_id);
                    //const spatial::EntityPtr& instantNextSubjectEndEntity = instant_next_sm.getEntity(subject_id);

                    //SpaceServer::SpaceMapPoint instant_next_subject_end_p(instant_next_md_subject_end.centerX,
                    //       instant_next_md_subject_end.centerY);

                    //double instant_next_dist = (instantNextObjectEntity->getPosition() - instantNextSubjectEndEntity->getPosition()).length( );//SpaceServer::SpaceMap::eucDist(instant_next_subject_end_p, instant_next_obj_p);

                    //std::cout << "NEAREST OBJECT AT THE NEXT INSTANT : " << obj_id
                    //      << " DIST : " << instant_next_dist << std::endl;
                    //~debug print

                    if (dist_subject_end > dist_ratio_threshold*sm.diagonalSize()) {
                        logger().debug("BE - the agent %s is at distance %f from the closest valide destination %s which is too far to be considered to have been reached", subject_id.c_str(), dist_subject_end, obj_id.c_str());
                        continue; //go directly to the next iteration
                        //and skip the creation of an elementary BD
                    } else {
                        logger().debug("BE - the agent %s is at distance %f from the closest valide destination %s which is close enough and therefore is considered to have been reached", subject_id.c_str(), dist_subject_end, obj_id.c_str());

                        //====
                        //create the arg list of goto_obj
                        //====
                        //get handle of goto_obj action
                        string goto_obj_name =
                            get_instance(id::goto_obj)->get_name();
                        Handle goto_obj_h = as.addNode(NODE, goto_obj_name);

                        //get handle of the destination object
                        std::list<Handle> ret;
                        as.getHandleSet(std::back_inserter(ret), OBJECT_NODE,
                                        obj_id, true);
                        OC_ASSERT(ret.size() == 1,
                                         "HandleSet should contain exactly one object.");
                        Handle obj_h = *ret.begin();
                        //get handle of speed
                        Handle speed_h = as.addNode(NUMBER_NODE,
                                                    boost::lexical_cast<string>(GOTO_OBJ_SPEED));

                        HandleSeq arg_seq{subject_h, goto_obj_h, obj_h, speed_h};
                            
                        new_arg_list_h = as.addLink(LIST_LINK, arg_seq);
                    } //~else

                } //~if(walk)
                else {//not a walk command
                    //we just need to flatten the action arguments
                    HandleSeq arg_seq{subject_h, per_h};
                    if (as.getArity(arg_list_h) > 2) {

                        Handle action_arg_list_h = as.getOutgoing(arg_list_h, 2);
                        OC_ASSERT(
                                         as.getType(action_arg_list_h) == LIST_LINK,
                                         "It is assumed that the arguments of the subject's action are wrapped in a listLink");

                        HandleSeq action_arg_seq = as.getOutgoing(action_arg_list_h);
                        arg_seq.insert(arg_seq.end(),
                                       action_arg_seq.begin(),
                                       action_arg_seq.end());
                    }
                    new_arg_list_h = as.addLink(LIST_LINK, arg_seq);
                }
                //if the instruction pointer arrives at that point of the code
                //it means either that the action being tracked is not a
                //walk command (cause it has not been taken the if branch above)
                //or it is a walk command but that is going to be converted
                //into an goto_obj (because otherwise an intruction continue;
                //that is would restart the loop)

                //Check that the action starts after the end of the previous one
                //and if not trim the beginning of it so that it does start after.
                if (previous_tu >= tl) {
                    tl = previous_tu + MinPauseTime;
                    //check that the previous action + MinPauseTime ends
                    //before the end of the current one + MinActionTime
                    OC_ASSERT(tu >= tl + MinActionTime,
                            "Apparently the previous action overlaps too much the current one and it's not clear which one should be first.");
                }

                HandleSeq el_seq{behaved_h, new_arg_list_h};
                Handle bd_h = as.addLink(EVALUATION_LINK, el_seq);
                Temporal t(tl, tu);
                Handle bd_t_h = as.getTimeServer().addTimeInfo(bd_h, t);

                //add member link
                HandleSeq memberLinkHS{bd_t_h, trickExemplarAtTime};
                as.addLink(MEMBER_LINK, memberLinkHS);

                logger().debug(
                             "BE - New elementary behavior description created: %s",
                             as.atomAsString(bd_t_h).c_str());


                //this in order to tell the converter walk to goto that we are not in
                //the middle of a walk to goto_obj convertion
                tl = 0;

                //tu is kept in memory so that we can check that
                //the action ends before the start of the next one
                previous_tu = tu;
            }
        }
    }*/
}

//perform a loop of updates from the beginning of the exemplar until the end
void BehaviorEncoder::updateRec(Temporal exemplarInterval)
{

    unsigned long tl = exemplarInterval.getLowerBound();
    unsigned long tu = exemplarInterval.getUpperBound();
    //get the list of temporals to perform the update
    std::vector<Temporal> temp_vec;
    unsigned long t1 = tl;
    unsigned long t2;
    do {
        t2 = std::min(t1 + time_resolution, tu);
        Temporal t(t1, t2);
        temp_vec.push_back(t);
        t1 = t2;
    } while (t2 < tu);



    for (vector<Temporal>::iterator ti = temp_vec.begin(); ti != temp_vec.end(); ++ti) {
        //Determine new_perception
        set<Handle> new_perceptions;
        vector<HandleTemporalPair> new_temps;
        //insert into new_temps all pairs (handle,temporal)
        //that have their starting time within t_grab_range
        timeServer().getTimeInfo(inserter(new_temps, new_temps.begin()),
                                       Handle::UNDEFINED,
                                       *ti,
                                       TemporalTable::STARTS_WITHIN);

        foreach(HandleTemporalPair htp, new_temps) {
            new_perceptions.insert(htp.getHandle());
        }

        foreach(const VFpair& vf, factories) {
            set<Handle> f_perceptions;
            copy_if(new_perceptions.begin(),
                             new_perceptions.end(),
                             inserter(f_perceptions, f_perceptions.begin()),
                             does_fit_template(vf.first, &wp->getAtomSpace()));

            vf.second->update(trickExemplarAtTime,
                              *ti,
                              f_perceptions.begin(),
                              f_perceptions.end());
        }
    }

}

bool BehaviorEncoder::update(Temporal start_moment)
{
    //wp->getAtomSpace()->print();

    if (!start_moment.getLowerBound() && !start_moment.getUpperBound())
        start_moment = next_moment;

    Temporal t_now(wp->getLatestSimWorldTimestamp());

//     printf("BE %s %s\n %lu %lu", next_moment.toString().c_str(), t_now.toString().c_str(), t_now.getUpperBound(),
//     next_moment.getUpperBound()+time_resolution);

    if (t_now.getUpperBound() < next_moment.getUpperBound() + time_resolution)
        return false;

    Temporal t_grab_range(start_moment.getLowerBound(), t_now.getLowerBound());

// printf("grab: %s\n", t_grab_range.toString().c_str());

    set<Handle> new_perceptions;
    vector<HandleTemporalPair> new_temps;
    //insert into new_temps all pairs (handle,temporal)
    //that have their starting time within t_grab_range
    timeServer().getTimeInfo(inserter(new_temps, new_temps.begin()),
                                   Handle::UNDEFINED,
                                   t_grab_range,
                                   TemporalTable::STARTS_WITHIN);

    foreach(HandleTemporalPair htp, new_temps) {
        //printf("new perc %s\n", htp.toString().c_str());
        new_perceptions.insert(htp.getHandle());
    }
    bprintf("Found %zu TOTAL perceptions\n", new_perceptions.size());
    bprintf("%zu factories\n", factories.size());

    foreach(const VFpair& vf, factories) {
        set<Handle> f_perceptions;
        copy_if(new_perceptions.begin(),
                         new_perceptions.end(),
                         inserter(f_perceptions, f_perceptions.begin()),
                         does_fit_template(vf.first, &wp->getAtomSpace()));

        bprintf("Found %zu valid perceptions\n", f_perceptions.size());

        vf.second->update(trickExemplarAtTime,
                          t_now,
                          f_perceptions.begin(),
                          f_perceptions.end());
    }

    //next_moment = Temporal(t_now.getUpperBound()+1, t_now.getUpperBound()+1);
    next_moment = t_now;
    //puts(t_now.toString().c_str());

    return true;
}

bool BehaviorEncoder::isUpdated() const
{
    return (wp->getLatestSimWorldTimestamp() < next_moment.getLowerBound() + time_resolution);
}

//takes in input a Handle that points to Eval
//and returns the Handle that points to Objname
//assuming the following structure
//Eval(Pred, List(Objname, ...))
Handle extractObjectID::operator()(Handle h) const
{
    Handle out1 = atomspace->getOutgoing(h, 0);
    if (atomspace->isNode(out1)) {
        Handle out21 = atomspace->getOutgoing(h, 1);
        if (atomspace->getArity(out21) > 0)
            return atomspace->getOutgoing(out21, 0);
    }

    OC_ASSERT(0, "Perception data syntax error!");
    return Handle::UNDEFINED;
}

BehaviorEncoder::~BehaviorEncoder()
{
}

} //namespace behavior
