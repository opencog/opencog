/*
 * opencog/embodiment/Learning/Filter/ActionFilter.cc
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


#include <opencog/util/exceptions.h>
#include <opencog/atomspace/Node.h>
#include <opencog/spacetime/atom_types.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/WorldWrapper/WorldWrapperUtil.h>

#include "ActionFilter.h"

namespace Filter
{

using namespace opencog::pai;
using namespace opencog::world;
using namespace PetCombo;

//constructor, destructor

ActionFilter::ActionFilter(const std::string& self_id,
                           const std::string& owner_id,
                           const WorldProvider& wp,
                           const definite_object_set& dos,
                           const indefinite_object_set& ios,
                           const builtin_action_set& bas,
                           arity_t arity,
                           bool type_check)
        : _self_id(self_id), _owner_id(owner_id), _wp(wp), _dos(dos),
        _ios(ios), _bas(bas), _arity(arity), _type_check(type_check)
{
}

ActionFilter::~ActionFilter() {}

//public methods

void ActionFilter::insertActionSubseqs(combo_tree_ns_set& actSubseq_set,
                                       const BehaviorCategory& bc,
                                       const argument_list_list& all,
                                       int max_size,
                                       bool only_min_max) const
{
    const std::vector<CompositeBehaviorDescription>& cbds = bc.getEntries();
    OC_ASSERT(all.size() == cbds.size(),
                     "There must be as many behavior descriptions"
                     " as argument lists");
    std::vector<CompositeBehaviorDescription>::const_iterator ie = cbds.begin();
    argument_list_list_const_it alci = all.begin();
    for (; ie != cbds.end(); ++ie, ++alci)
        insertActionSubseqs(actSubseq_set, *ie, *alci,
                            max_size, only_min_max);
}

void ActionFilter::insertActionSubseqs(combo_tree_ns_set& actSubseq_set,
                                       const CompositeBehaviorDescription& cbd,
                                       const argument_list& al,
                                       int max_size,
                                       bool only_min_max) const
{
    //determine complete action sequence
    combo_tree_ns_set completeSeq_set = generateCompleteActionSeqs(cbd, al);

    //then select all sub sequences of size <=max_size
    OC_ASSERT(max_size > 0 || max_size == -1,
                     "You wanna select no action? Don't use an action"
                     " filter then!");
    for (combo_tree_ns_set_const_it ias = completeSeq_set.begin();
            ias != completeSeq_set.end(); ++ias) {

        //sub_seq_size is the number of actions of *ias
        unsigned sub_seq_size;
        pre_it head = ias->begin();
        if (*head == id::sequential_and)
            sub_seq_size = head.number_of_children();
        else sub_seq_size = 1;

        //determine max_size for ias, noted ms
        unsigned ms;
        if (max_size == -1) {
            ms = sub_seq_size;
        }
        else {
            OC_ASSERT(max_size >= 0);
            ms = std::min((unsigned)max_size, sub_seq_size);
        }

        //loop over each size from 1 to ms
        //Note: that if only_min_max is true then s jumps directly to ms
        // (see the code at the end of the loop)
        for (unsigned s = 1; s <= ms; s++) {

            if (*head == id::sequential_and) {
                if (ias->number_of_children(head) >= s) {
                    if (s == 1) {
                        for (sib_it sib = head.begin();
                                sib != head.end(); ++sib) {
                            combo_tree tmp(sib);
                            actSubseq_set.insert(tmp);
                        }
                    } else {
                        sib_it from = head.begin();
                        //last correspond to the last sibling included in the sequence
                        sib_it last = from;
                        last += s - 1;
                        for (; last != head.end(); ++from, ++last) {
                            combo_tree tmp;
                            tmp.set_head(id::sequential_and);
                            pre_it tmp_head = tmp.begin();
                            tmp.append_children(tmp_head, s);
                            tmp.replace(tmp_head.begin(), tmp_head.end(),
                                        from, ++sib_it(last));
                            actSubseq_set.insert(tmp);
                        }
                    }
                }
            } else {
                OC_ASSERT(is_builtin_action(*head),
                                 "If head is not and_seq them must be"
                                 " single builtin_action");
                if (s == 1) actSubseq_set.insert(*ias);
            }
            
            //if only_min_max is true then jump directly to the max case
            //(the min case, that is s == 1, has just been taken care of)
            if(only_min_max && s < ms)
                s = ms-1;
        }
    }
}

ActionFilter::combo_tree_ns_set ActionFilter::generateCompleteActionSeqs(
        const CompositeBehaviorDescription& cbd,
        const argument_list& al) const
{
    combo_tree_ns_set compActionSeq;
    completeActionPrefixes(compActionSeq, cbd, al, 0);
    return compActionSeq;
}

void ActionFilter::completeActionPrefixes(combo_tree_ns_set& actPrefix_set,
        const CompositeBehaviorDescription& cbd,
        const argument_list& al,
        unsigned index) const
{
    const std::vector<PredicateHandleSet>& tls = cbd.getTimelineSets();
    if (index < tls.size()) {
        const PredicateHandleSet& phs = tls[index];
        unsigned s = phs.getSize();
        OC_ASSERT(s == 0 || s == 1,
                         "There must is 0 or 1 action in phs");
        if (s > 0) {
            Handle h = *phs.getSet().begin();
            combo_tree_ns_set act_set;
            //get current time and a spaceMapHandle associated
            unsigned long time = cbd.getIndexStartTime(index);
            Handle smh =
                AtomSpaceUtil::getCurrentSpaceMapHandle(_wp.getAtomSpace());

            OC_ASSERT(smh != Handle::UNDEFINED,
                             "There must be a SpaceMap, ask Nil for more.");
            generatePossibleActions(act_set, h, al, smh, time);
            //when actPrefix_set is empty
            if (actPrefix_set.empty()) {
                combo_tree tmp(id::sequential_and);
                actPrefix_set.insert(tmp);
            }
            //base case
            //local copy in order to iterate over the set of prefix
            //without meesing up with additional prefixes
            //in the course of the algo
            combo_tree_ns_set ps_copy = actPrefix_set;
            actPrefix_set.clear();
            for (combo_tree_ns_set_it vsi = ps_copy.begin();
                    vsi != ps_copy.end(); ++vsi) {
                combo_tree prefix = *vsi;
                pre_it prefix_head = prefix.begin();
                OC_ASSERT(
                                 *prefix_head == id::sequential_and,
                                 "It should be assumed that prefix_head"
                                 " is sequential_and");
                for (combo_tree_ns_set_it ai = act_set.begin();
                        ai != act_set.end(); ++ai) {
                    OC_ASSERT(!ai->empty(),
                                     "ai should not be empty");
                    combo_tree tmp(prefix_head);
                    tmp.replace(tmp.append_child(tmp.begin()), ai->begin());
                    actPrefix_set.insert(tmp);
                }
            }
        }
        //recursive case
        completeActionPrefixes(actPrefix_set, cbd, al, index + 1);
    }
}

void ActionFilter::generatePossibleActions(combo_tree_ns_set& act_set,
        Handle h,
        const argument_list& al,
        Handle smh,
        unsigned long time) const
{
    const AtomSpace& as = _wp.getAtomSpace();

    OC_ASSERT( h != Handle::UNDEFINED,
                 "h must conrrespond to a defined handle");
    OC_ASSERT( as.getType(h) == EVALUATION_LINK,
                 "ActionFilter - h should be of type EVALUATION_LINK.");

    //get the list of args
    Handle list_arg_h = as.getOutgoing(h, 1);

    OC_ASSERT(list_arg_h != Handle::UNDEFINED,
                 "list_arg_h must correspond to a defined handle");
    OC_ASSERT( as.getType(list_arg_h) == LIST_LINK,
                 "ActionFilter - h outgoing atom 1 should be of type LIST_LINK. ");

    //get finally action name
    Handle action_h = as.getOutgoing(list_arg_h, 1);

    OC_ASSERT( as.isNode(action_h),
                 "ActionFilter - action handle should inherits from NODE");

    //get the action name
    std::string action_name = (as.getName(action_h));

    // print debug
    std::cout << "action_name = " << action_name << std::endl;

    // Determine whether the action is in the action_set.
    // Be sure to use the PetCombo operator>> for this, as otherwise,
    // the compiler will pick the wrong one, and/or get confused.
    vertex head_v;
    stringstream ss(action_name);
    // ss >> head_v;
    PetCombo::operator>>(ss,head_v);


    // print debug
    std::cout << "head_v = " << head_v << std::endl;

    OC_ASSERT(is_builtin_action(head_v),
                     "%s is not a builtin_action", action_name.c_str());

    if (_bas.find(get_builtin_action(head_v)) != _bas.end()) {
        //get arg list names and generate all possible list of arguments
        std::vector<vertex> operand_list;
        int list_arg_arity = as.getArity(list_arg_h);

        if (list_arg_arity == 2) { //check if the action has no argument
            act_set.insert(combo_tree(head_v));
        } else { //the action has arguments
            for (int i = 2; i < list_arg_arity; ++i) {
                Handle arg_h = as.getOutgoing(list_arg_h, i);

                Type arg_type = as.getType(arg_h);
                const string& arg_name = as.getName(arg_h);
                //check if the argument is an identifier
                if (classserver().isA(arg_type, OBJECT_NODE)) {
                    vertex arg_v =
                        WorldWrapperUtil::atom_name_to_definite_object(arg_name,
                                _self_id,
                                _owner_id);
                    operand_list.push_back(arg_v);
                }
                //check if the argument is a number
                else if (classserver().isA(arg_type, NUMBER_NODE)) {
                    vertex arg_v = boost::lexical_cast<contin_t>(arg_name);
                    operand_list.push_back(arg_v);
                }
                //that case is not handle yet
                else {
                    OC_ASSERT(false,
                            "Only identifier and number are accepted as argument");
                }
            }

            std::set< std::vector<vertex> > operand_list_set;
            generatePossibleOperands(operand_list_set, operand_list,
                                     al, smh, time);

            //generate all possible actions by varying all possible operands
            for (std::set< std::vector<vertex> >::const_iterator vsi
                    = operand_list_set.begin(); vsi != operand_list_set.end();
                    ++vsi) {
                combo_tree tmp;
                tmp.set_head(head_v);
                pre_it head = tmp.begin();
                for (std::vector<vertex>::const_iterator ai = vsi->begin();
                        ai != vsi->end(); ++ai) {
                    tmp.append_child(head, *ai);
                }
                act_set.insert(tmp);
            }
        }
    } else {
        logger().warn(
                     "ActionFilter - builtin action '%s' is not in the list"
                     " _bas of action of interest and will be ignored",
                     action_name.c_str());
    }

    //type check the generated actions
    if (_type_check) {
        for (combo_tree_ns_set_const_it ati = act_set.begin();
                ati != act_set.end(); ++ati) {
            type_tree tt = infer_type_tree(*ati);
            stringstream ss_tr;
            ss_tr << *ati;
            OC_ASSERT(is_well_formed(tt),
                             "Action combo tree '%s' is not well formed,"
                             " see the log for more information",
                             ss_tr.str().c_str());
        }
    }
}

void ActionFilter::generatePossibleOperands(std::set< std::vector<vertex> >& opras_set,
        const std::vector<vertex>& ovl,
        const argument_list& al,
        Handle smh,
        unsigned long time) const
{
    OC_ASSERT(opras_set.empty(),
                     "opars_set must be empty");
    completePrefixOperands(opras_set, 0, ovl, al, smh, time);
}

void ActionFilter::completePrefixOperands(std::set< std::vector<vertex> >& prefix_opras,
        unsigned index,
        const std::vector<vertex>& ovl,
        const argument_list& al,
        Handle smh,
        unsigned long time) const
{
    if (index < ovl.size()) {
        const vertex operand = ovl[index];
        std::set<vertex> opra_set;
        generatePossibleOperands(opra_set, operand, al, smh, time);
        //base case
        if (prefix_opras.empty()) {
            foreach(const vertex& v, opra_set) {
                std::vector<vertex> vv(1, v);
                prefix_opras.insert(vv);
            }
        } else {
            std::set< std::vector<vertex> > prefix_opras_copy = prefix_opras;
            prefix_opras.clear();
            foreach(std::vector<vertex> vv, prefix_opras_copy) {
                foreach(const vertex& v, opra_set) {
                    vv.push_back(v);
                    prefix_opras.insert(vv);
                }
            }
        }
        //recursive case
        completePrefixOperands(prefix_opras, index + 1, ovl, al, smh, time);
    }
}

void ActionFilter::generatePossibleOperands(std::set<vertex>& opras,
        const vertex& operand,
        const argument_list& al,
        Handle smh,
        unsigned long time) const
{
    //fist of all add the operand itself
    opras.insert(operand);

    //definite_object case
    if (is_definite_object(operand)) {
        definite_object defo = get_definite_object(operand);
        OC_ASSERT(_dos.find(defo) != _dos.end(),
                         "The definite object %s found in the behavior"
                         " descriptions is not in the set of definite object"
                         " of interest, there must be something wrong"
                         " happening with the Behavior Encoder/Tracker",
                         defo.c_str());

        //add the indefinite objects
        for (indefinite_object_set_const_it ioi = _ios.begin();
                ioi != _ios.end(); ++ioi) {
            indefinite_object io = *ioi;

            // random_object case
            if (is_random(io)) {
                if(io == id::random_object)
                    opras.insert(io);
                //check that the object could be such random indefinite object
                else {
                    perception is_X =
                        WorldWrapperUtil::nearest_random_X_to_is_X(io);
                    pre_it it =
                        WorldWrapperUtil::maketree_percept(is_X, defo);
                    bool corresponding_arg_exists =
                        combo::vertex_to_bool(WorldWrapperUtil::evalPerception
                                              (smh,
                                               time,
                                               _wp.getAtomSpace(),
                                               _self_id, _owner_id,
                                               it,
                                               true));
                    if (corresponding_arg_exists)
                        opras.insert(io);
                }
            }
            //non random object case
            //evaluate the indefinite object and check that it matches obj_name
            else {
                vertex v = WorldWrapperUtil::evalIndefiniteObject(
                           smh,
                           time,
                           _wp.getAtomSpace(),
                           _self_id,
                           _owner_id,
                           io,
                           true);
                if (v == defo)
                    opras.insert(io);
            }
        }
    }
    //all other cases (definite_object and contin)
    //add all variables (input arguments) that match operand
    for (int idx = 1; idx <= _arity; idx++) {
        if (al[idx-1] == operand)
            opras.insert(argument(idx));
    }
}

}//~namespace Filter
