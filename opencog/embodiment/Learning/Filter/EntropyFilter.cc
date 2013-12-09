/*
 * opencog/embodiment/Learning/Filter/EntropyFilter.cc
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

//just for debug and profiling
#include <time.h>

#include <opencog/util/exceptions.h>
#include <opencog/util/misc.h>
#include <opencog/util/numeric.h>
#include <opencog/util/lru_cache.h>

#include <opencog/comboreduct/type_checker/type_tree.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/TimeServer.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/WorldWrapper/WorldWrapperUtil.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>
#include <opencog/embodiment/AtomSpaceExtensions/CompareAtomTreeTemplate.h>
#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>

#include "EntropyFilter.h"

//this number defines the maximum number of optional input arguments
//(that is in addition to the mandatory ones) when the perception
//uses arg_list(T) as last type argument
#define MAX_OPTIONAL_ARGUMENTS 2

//The option below is deprecated
//when this flag is on it computes an approximation of the entropy
//of perceptions involving random operators instead of the exact entropy
//NOTE: exact random entropy takes too many resources
//#define OPTIMIZE_RANDOM_ENTROPY true

//this correspond to the granularity to evaluate is_last_action_action
//in time unit
#define IS_LAST_AGENT_ACTION_DELAY 50

namespace Filter
{

using namespace opencog::pai;
using namespace opencog::world;
using namespace PetCombo;
using namespace opencog; /// @todo make it under the namespace opencog

//constructor, destructor
EntropyFilter::EntropyFilter(const std::string& self_id,
                             const std::string& owner_id,
                             AtomSpace& atomSpace,
                             const perception_set& ep,
                             const indefinite_object_set& idos,
                             const definite_object_set& dos,
                             const message_set& ms,
                             const agent_to_actions& atas,
                             const type_tree_seq& input_arg_types)
        : _self_id(self_id), _owner_id(owner_id),
        _atomSpace(atomSpace), _elementary_perceptions(ep),
        _idos(idos), _dos(dos), _ms(ms), _atas(atas),
        _input_arg_types(input_arg_types),
        _total_time(0),
        _indefToDef(id::avatar_indefinite_object_count, "")
{

    _arity = _input_arg_types.size();

    //init _hasSaidDelay
    _hasSaidDelay = WorldWrapperUtil::getHasSaidDelay();

    //init perceptToTime

    //build operand set (perception tree children)
    build_operand_set();

    //build perceptToTime
    //(or perceptToBoolTime depending on whether optimization is enabled)
    for (perception_set_const_it psi = _elementary_perceptions.begin();
            psi != _elementary_perceptions.end(); ++psi) {
        build_and_insert_atomic_perceptions(*psi);
    }

    //init spaceMapNode

    _spaceMapNode = spaceServer().getLatestMapHandle();
    OC_ASSERT(_spaceMapNode != Handle::UNDEFINED,
                     "There must a be a map node in the atomSpace");
}

EntropyFilter::~EntropyFilter() {}

void EntropyFilter::generateFilteredPerceptions(combo_tree_ns_set& pred_set,
        double threshold,
        const BehaviorCategory& BDCat,
        const std::vector<Temporal>& est,
        const argument_list_list& all)
{
    const std::vector<CompositeBehaviorDescription>& bdce = BDCat.getEntries();
    OC_ASSERT(!bdce.empty(), "Error : No exemplars");
    OC_ASSERT(_arity == 0 || all.size() == bdce.size(),
                     "If the program to learn has a non-null arity then the number of argument lists must be equal to the number of behavior descriptions");
    argument_list_list_const_it alli = all.begin();
    std::vector<CompositeBehaviorDescription>::const_iterator bdi = bdce.begin();
    std::vector<Temporal>::const_iterator eti = est.begin();
    for (; bdi != bdce.end(); ++bdi, ++alli, ++eti) {
        OC_ASSERT(!bdi->empty(),
                         "Exemplar should not be empty for now, ask Nil to fix that");
        updatePerceptToTime(*eti, *alli);
    }
    generateFilteredPerceptions(pred_set, threshold);
}

void EntropyFilter::generateFilteredPerceptions(combo_tree_ns_set& pred_set,
        double threshold,
        const CompositeBehaviorDescription&
        cbd,
        const Temporal& et,
        const argument_list& al)
{
    rebuildPerceptToTime();
    updatePerceptToTime(et, al);
    generateFilteredPerceptions(pred_set, threshold);
}

void EntropyFilter::updatePerceptToTime(const Temporal& temp,
                                        const argument_list& al)
{
    unsigned long tl = temp.getLowerBound();
    unsigned long tu = temp.getUpperBound();
    long diff = (long)tu - (long)tl;
    OC_ASSERT(diff >= 0, "diff = %d is not positive or null", diff);
    //get the list of spaceMap that occurs in that range
    std::vector<HandleTemporalPair> htps;
    //get the first map at tl or if not before tl
    Temporal temp_right_after(tl + 1, tu);
    timeServer().getTimeInfo(back_inserter(htps), _spaceMapNode, temp_right_after,
                           TemporalTable::PREVIOUS_BEFORE_START_OF);
    OC_ASSERT(!htps.empty(),
                     "There must be a map that starts at %d or at least before %d",
                     tl, tl);
    //try to get the map
    // get temporal pairs that start within temp_right_after, to not get
    //twice the first map
    timeServer().getTimeInfo(back_inserter(htps), _spaceMapNode, temp_right_after,
                           TemporalTable::STARTS_WITHIN);

    const SpaceServer::SpaceMap* pre_sm = NULL; //previous spaceMap
    //used for isMoving
    //for each spaceMap update _perceptToTime
    for (std::vector<HandleTemporalPair>::const_iterator htp_it = htps.begin();
            htp_it != htps.end(); ++htp_it) {
        //determine spaceMap
        Handle smh = timeServer().getAtTimeLink(*htp_it);
        OC_ASSERT(smh != Handle::UNDEFINED,
                         "There must be a spaceMap for that handle");
        const SpaceServer::SpaceMap& sm = spaceServer().getMap(smh);
        //determine lower and upper boundary of that spaceMap
        //if the space map started before the exemplar start time
        //ltl is the exemplar start time instead
        //and there is no next spaceMap then ltu is the exemplar stop time
        Temporal ltemp = *htp_it->getTemporal();
        unsigned long ltl = htp_it->getTemporal()->getLowerBound();
        if (ltl < tl)
            ltl = tl;
        unsigned long ltu;
        std::vector<HandleTemporalPair>::const_iterator htp_it_next = htp_it;
        ++htp_it_next;
        if (htp_it_next != htps.end()) {
            ltu = htp_it_next->getTemporal()->getLowerBound();
            OC_ASSERT(ltu <= tu, "The start time of the last spaceMap must occur at or before the exemplar stop time");
        } else ltu = tu;
        unsigned long ldiff = ltu - ltl;
#ifdef ISMOVING_OPTIMIZE
        //compute isMoving for all object, if the object does not
        //belong to the map yet it goes with true, because we don't
        //have previous value of the predicate at this point
        foreach(const definite_object& obj, _dos)
            setIsMoving(obj, pre_sm, sm);
        //eval each perception
        for (combo_tree_bool_time_map_it vti = _perceptToBoolTime.begin();
                vti != _perceptToBoolTime.end(); ++vti)
#else
        for (combo_tree_time_map_it vti = _perceptToTime.begin();
                vti != _perceptToTime.end(); ++vti)
#endif
        {
            bool isMovOptPossible = false; //isMoving optimization
            //is potentially possible
            combo_tree tmp = vti->first; //a copy is performed because the argument
            //might be changed
            pre_it head_it = tmp.begin();
            vertex head = *head_it;
            //evaluate perception operand
            for (sib_it opra = head_it.begin(); opra != head_it.end(); ++opra) {
                //evaluate perception operand if indefinite object
                if (is_indefinite_object(*opra)) {
                    indefinite_object io = get_indefinite_object(*opra);
                    avatar_indefinite_object_enum ioe = get_enum(io);
                    //check if the indefinite object is in _indefToDef cache
                    if (_indefToDef[(unsigned int)ioe] == "") {
                        *opra =
                            WorldWrapperUtil::evalIndefiniteObject(smh,
                                                                   ltl,
                                                                   _atomSpace,
                                                                   _self_id,
                                                                   _owner_id,
                                                                   io,
                                                                   true /*isInThePast*/);
                        //only non random indefinite objects go in the cache
                        if (!is_random(io)) {
                            OC_ASSERT(is_definite_object(*opra),
                                             "opra must contain a definite_object");
                            _indefToDef[(unsigned int)ioe] = get_definite_object(*opra);
                        } else { //is moving optimization cannot work with random object
                            isMovOptPossible = false;
                        }
                    } else *opra = vertex(_indefToDef[(unsigned int)ioe]);
                }
                //if operand is function argument
                else if (is_argument(*opra))
                    *opra = al[get_argument(*opra).abs_idx_from_zero()];
            }
#ifdef ISMOVING_OPTIMIZE
            //check if the perception is is_moving then look at _isMoving
            if (head == get_instance(id::is_moving)) {
                OC_ASSERT(head_it.has_one_child(),
                                 "is_moving must have only one child");
                //eval the new perception
                std::pair<bool, unsigned long>& p = vti->second;
                //look into the isMoving cache
                OC_ASSERT(is_definite_object(*head_it.begin()),
                                 "the argument of is_moving must be a definite object");
                p.first = getIsMoving(get_definite_object(*head_it.begin()));
                if (p.first)
                    p.second += ldiff;
            }
            //hasSaid is particular because we must cut the time in
            //hasSaidDelay time intervals between the start and the end of
            //the spaceMap
            else if (head == get_instance(id::has_said)) {
                OC_ASSERT(_hasSaidDelay > 0,
                                 "_hasSaidDelay cannot be null otherwise that may provoke an infinite loop");
                for (unsigned long i = ltu; i > ltl; i -= _hasSaidDelay) {
                    if (combo::vertex_to_bool(WorldWrapperUtil::evalPerception(
                                              smh,
                                              i,
                                              _atomSpace,
                                              _self_id,
                                              _owner_id,
                                              head_it,
                                              true))) {
                        unsigned long idiff;
                        if (i - _hasSaidDelay > ltl)
                            idiff = _hasSaidDelay;
                        else idiff = i - ltl;
                        std::pair<bool, unsigned long>& p = vti->second;
                        p.second += idiff;
                    }
                }
            }
            //this code is very badly optimized it should be optimized later
            else if (head == get_instance(id::is_last_agent_action)) {

                //debug print
                //std::cout << "PERCEPTION IS_LAST_AGENT_ACTION : " << combo_tree(head_it) << std::endl;
                //~debug print

                unsigned int t = ltl;

                //retreive all actions of the agent involved in the perception
                //in time interval of the SpaceMap
                std::list<HandleTemporalPair> htp;
                timeServer().getTimeInfo(back_inserter(htp),
                                       Handle::UNDEFINED,
                                       Temporal(ltl, ltu), TemporalTable::ENDS_WITHIN);

                pre_it head_child_it = head_it.begin();
                Handle action_done_h = _atomSpace.getHandle(PREDICATE_NODE,
                                       ACTION_DONE_PREDICATE_NAME);
                Handle agent_h =
                    WorldWrapperUtil::toHandle(_atomSpace, get_definite_object(*head_child_it),
                                               _self_id, _owner_id);
                //define template to match
                atom_tree* no_arg_actionDone = makeVirtualAtom(EVALUATION_LINK,
                                               makeVirtualAtom(action_done_h, NULL),
                                               makeVirtualAtom(LIST_LINK, makeVirtualAtom(agent_h, NULL), NULL),
                                               NULL);
                does_fit_template dft(*no_arg_actionDone, &_atomSpace, false);
                for (std::list<HandleTemporalPair>::const_iterator i = htp.begin();
                        i != htp.end(); ++i) {
                    Handle evalLink_h = i->getHandle();
                    //check if evalLink_h match the template
                    if (dft(evalLink_h)) {
                        unsigned long cur_tu = i->getTemporal()->getUpperBound();
                        if (combo::vertex_to_bool(WorldWrapperUtil::evalPerception(
                                                  smh,
                                                  cur_tu,
                                                  _atomSpace,
                                                  _self_id,
                                                  _owner_id,
                                                  head_it,
                                                  true))) {
                            std::pair<bool, unsigned long>& p = vti->second;
                            p.second += cur_tu - t;
                        }
                        t = cur_tu + 1;
                    }
                }
            }
            //check if we can use the previous value rather than computing
            //a new one
            else {
                bool canUsePreviousValue;
                if (isMovOptPossible && doesInvolveMoving(head)) {
                    canUsePreviousValue = true;
                    for (sib_it opra = head_it.begin();
                         opra != head_it.end() && canUsePreviousValue; ++opra)
                        canUsePreviousValue =
                            !getIsMoving(get_definite_object(*opra));
                } else canUsePreviousValue = false;
                //eval the new perception
                std::pair<bool, unsigned long>& p = vti->second;
                if (!canUsePreviousValue) {
                    p.first =
                        combo::vertex_to_bool(WorldWrapperUtil::evalPerception(smh,
                                              ltl,
                                              _atomSpace,
                                              _self_id,
                                              _owner_id,
                                              head_it,
                                              true));
                }
                if (p.first)    // use the previous value (isMoving
                                // optimization)
                    p.second += ldiff;
            }

#else //~ISMOVING_OPTIMIZE

            if (head == get_instance(id::has_said)) {
                for (unsigned long i = ltu; i > ltl; i -= _hasSaidDelay) {
                    if (combo::vertex_to_bool(WorldWrapperUtil::evalPerception(
                                              smh,
                                              i,
                                              _atomSpace,
                                              _self_id,
                                              _owner_id,
                                              head_it,
                                              true))) {
                        unsigned long idiff;
                        if (i - _hasSaidDelay > ltl)
                            idiff = _hasSaidDelay;
                        else idiff = i - ltl;
                        vti->second += idiff;
                    }
                }
            } else {
                if (combo::vertex_to_bool(WorldWrapperUtil::evalPerception(
                                          smh,
                                          ltl,
                                          _atomSpace,
                                          _self_id,
                                          _owner_id,
                                          head_it,
                                          true)))
                    vti->second += ldiff;
            }

#endif
        }
        //get the pointer of the previous spaceMap for the next iterator,
        //for isMoving
        pre_sm = &sm;
        //reset _indefToDef
        for (unsigned int i = 0; i < _indefToDef.size(); i++)
            _indefToDef[i] = "";
    }
    _total_time += diff;
}

void EntropyFilter::updatePerceptToTime(unsigned long tl, unsigned long tu,
                                        const argument_list& al)
{
    updatePerceptToTime(Temporal(tl, tu), al);
}

void EntropyFilter::rebuildPerceptToTime()
{
    //rebuild argument set (insert new possible arguments)
    build_operand_set(); //no need to clear the set because all possible operands are kept anyway

    //add new atomic perceptions
    for (perception_set_const_it psi = _elementary_perceptions.begin();
            psi != _elementary_perceptions.end(); ++psi) {
        build_and_insert_atomic_perceptions(*psi);
    }
}

//fill pred_set with all predicates with entropy above threshold
void EntropyFilter::generateFilteredPerceptions(combo_tree_ns_set& pred_set,
                                                double threshold)
{
#ifdef ISMOVING_OPTIMIZE
    for (combo_tree_bool_time_map_it is = _perceptToBoolTime.begin();
         is != _perceptToBoolTime.end(); ++is) {
        std::pair<bool, unsigned long>& pa = is->second;
        float p = (float)pa.second / (float)_total_time;
        float entropy = opencog::binaryEntropy(p);
        //print debug
        //std::cout << "PERCEPTION TR : " << is->first << " P : " << p
        // << " ENTROPY : " << entropy << std::endl;
        //~print debug
        if (entropy > threshold)
            pred_set.insert(is->first);
    }
#else
    for (combo_tree_time_map_it is = _perceptToTime.begin();
            is != _perceptToTime.end(); ++is) {
        float p = (float)is->second / (float)_total_time;
        float entropy = opencog::binaryEntropy(p);
        if (entropy > threshold)
            pred_set.insert(is->first);
    }
#endif
}

inline bool EntropyFilter::doesInvolveMoving(vertex v)
{
    return (v == get_instance(id::near) || v == get_instance(id::below)
            || v == get_instance(id::above) || v == get_instance(id::inside));
}

inline void EntropyFilter::setIsMoving(const definite_object& obj, bool val)
{
#ifdef ISMOVING_SET
    if(val)
        _isMoving.insert(obj);
    else
        _isMoving.erase(obj);
#else
    _isMoving[obj] = val;
#endif
}

inline void EntropyFilter::setIsMoving(const definite_object& obj,
                                       const SpaceServer::SpaceMap* pre_sm,
                                       const SpaceServer::SpaceMap& sm)
{
    if (pre_sm == NULL)
        setIsMoving(obj, true);
    else {
        Handle obj_h = WorldWrapperUtil::toHandle(_atomSpace, obj,
                                                  _self_id, _owner_id);
        setIsMoving(obj, AtomSpaceUtil::isMovingBtwSpaceMap(_atomSpace,
                                                            *pre_sm, sm,
                                                            obj_h));
    }
}

inline bool EntropyFilter::getIsMoving(const definite_object& obj)
{
#ifdef ISMOVING_SET
    return _isMoving.end() != _isMoving.find(obj);
#else
    return _isMoving[obj];
#endif
}

void EntropyFilter::build_operand_set()
{
    for (definite_object_set_const_it do_it = _dos.begin();
            do_it != _dos.end(); ++do_it) {
        _operands.insert(*do_it);
    }
    for (indefinite_object_set_const_it io_it = _idos.begin();
            io_it != _idos.end(); ++io_it) {
        _operands.insert(*io_it);
    }
    for (message_set_const_it ms_it = _ms.begin();
            ms_it != _ms.end(); ++ms_it) {
        _operands.insert(*ms_it);
    }
    //for(agent_to_actions_const_it atas_it = _atas.begin();
    //atas_it != _atas.end(); ++atas_it) {
    // for(definite_object_vec_set_const_it ados_it = atas_it->second.begin();
    //  ados_it != atas_it->second.end(); ++ados_it) {
    //_operands.insert(*ados_it);
    //}
    //}
    for (arity_t i = 1; i <= _arity; i++) {
        argument arg(i);
        _operands.insert(arg);
    }
}

void EntropyFilter::build_and_insert_atomic_perceptions(perception p)
{
    arity_t p_arity = p->arity();
    combo_tree p_tr(p);
    if (p_arity < 0) {
        p_arity = -p_arity;
        for (arity_t a = p_arity - 1; a < p_arity + MAX_OPTIONAL_ARGUMENTS; a++)
            build_and_insert_atomic_perceptions(p_tr, a);
    } else build_and_insert_atomic_perceptions(p_tr, p_arity);
}

void EntropyFilter::build_and_insert_atomic_perceptions(const combo_tree& tr,
                                                        arity_t arity)
{
    OC_ASSERT(arity >= 0, "arity is necessarily positive or null");
    OC_ASSERT(!tr.empty(), "tr cannot be empty");
    pre_it head = tr.begin();

    //debug print
    //std::cout << "BUILD_AND_INSER TR : " << tr
    //      << " ARITY : " << (int)arity << std::endl;
    //~debug print

    //---------
    //base case
    //---------
    if (arity == 0) {
        combo_tree tmp = tr;
        reduct::hillclimbing_perception_reduce(tmp);
        pre_it tmp_head = tmp.begin();
        if (*tmp_head != id::logical_true && *tmp_head != id::logical_false) {
#ifdef ISMOVING_OPTIMIZE
            //if the perception is new then add it
            //this conditional is used because
            //it might not be the first time the set of atomic perception
            //is being built (when other new BD exemplars comes for instance)
            if (_perceptToBoolTime.find(tmp) == _perceptToBoolTime.end()) {
                std::pair<bool, unsigned long> p(false, 0);
                _perceptToBoolTime[tmp] = p;
                //print debug
                //std::cout << "ADD PERCEPT TO TIME : " << tmp << std::endl;
                //~print debug
            }
#else
            //if the perception is new then add it
            //this conditional is used because
            //it might not be the first time the set of atomic perception
            //is being built (when other new BD exemplars comes for instance)
            if (_perceptToTime.find(tmp) == _perceptToTime.end())
                _perceptToTime[tmp] = 0;
#endif
        }
    }
    //--------------
    //recursive case
    //--------------
    //is_last_agent_action is a special case because it uses
    //_atas (containing a map of agents corresponding to there
    //actions and arguments) instead of _operands
    else if (*head == get_instance(id::is_last_agent_action)) {

        //debug print
        //std::cout << "TR HEAD : " << combo_tree(head)
        //    << " VERTEX : " << *v_it
        //    << std::endl;
        //~debug print

        for (agent_to_actions_const_it aaci = _atas.begin();
                aaci != _atas.end(); ++aaci) {
            for (definite_object_vec_set_const_it dci = aaci->second.begin();
                    dci != aaci->second.end(); ++dci) {
                combo_tree tmp = tr;
                pre_it tmp_head = tmp.begin();
                OC_ASSERT(tmp_head.is_childless(),
                                 "to work well there must be no child");
                tmp.append_child(tmp_head, aaci->first);
                for (definite_object_vec_const_it dvci = dci->begin();
                        dvci != dci->end(); ++dvci) {
                    tmp.append_child(tmp_head, *dvci);
                }
                build_and_insert_atomic_perceptions(tmp, 0);
            }
        }
    } else {
        //get the type tree of the next argument
        arity_t noc = head.number_of_children();
        type_tree tt = get_input_type_tree(*head, noc);

        //iterate over all possible operands and include
        //only those that inherit
        //from the next argument type tree tt
        for (vertex_set_const_it v_it = _operands.begin();
                v_it != _operands.end(); ++v_it) {
            type_tree v_tt;
            if (is_argument(*v_it)) {//return the type corresponding to argument
                const argument arg = get_argument(*v_it);
                v_tt = _input_arg_types[arg.abs_idx_from_zero()];
            } else v_tt = get_type_tree(*v_it);

            //if inherits then create a new combo_tree with the new argument
            //and call recursively with arity-1
            if (inherit_type_tree(v_tt, tt)) {
                combo_tree tr_copy = tr;
                pre_it head_copy = tr_copy.begin();
                tr_copy.append_child(head_copy, *v_it);
                build_and_insert_atomic_perceptions(tr_copy, arity - 1);
            }
        }
    }
}
}
