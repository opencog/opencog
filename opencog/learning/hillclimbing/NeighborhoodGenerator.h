#ifndef _HILLCLIMBING_NEIGHBORHOODGENERATOR_H
#define _HILLCLIMBING_NEIGHBORHOODGENERATOR_H

#include "util/exceptions.h"
#include "util/Logger.h"

#include "comboreduct/combo/vertex.h"
#include "comboreduct/combo/type_tree.h"
#include "comboreduct/reduct/reduct.h"

//apply reduction for all generated programs
#define APPLY_FULL_REDUCTION

//apply reduction for all action type elementary sub-programs
#define APPLY_ACTION_REDUCTION

//apply permutation neighborhood expansion,
//that is and_seq(A1 A2 A3) is neighbor with
//and_seq(A2 A1 A3), and_seq(A2 A3 A1), and_seq(A1 A3 A2), and_seq(A3 A1 A2)
#define PERMUTATION_NEIGHBORHOOD_EXPANSION

namespace hillclimbing
{

using namespace combo;
using namespace opencog;
using namespace reduct;

//ns = normal size
typedef std::set<combo_tree, opencog::size_tree_order<vertex> > combo_tree_ns_set;
typedef combo_tree_ns_set::iterator combo_tree_ns_set_it;
typedef combo_tree_ns_set::const_iterator combo_tree_ns_set_const_it;

typedef std::set<vertex> operator_set;
typedef operator_set::iterator operator_set_it;
typedef operator_set::const_iterator operator_set_const_it;

typedef std::set<vertex> object_set;
typedef object_set::iterator object_set_it;
typedef object_set::const_iterator object_const_set_it;

template < typename Combo_TreeComp = opencog::size_tree_order<vertex> >
class NeighborhoodGenerator
{

    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::iterator pre_it;

    typedef std::set<combo_tree, Combo_TreeComp> combo_tree_set;
    typedef typename combo_tree_set::iterator combo_tree_set_it;
    typedef typename combo_tree_set::const_iterator combo_tree_set_const_it;

    typedef combo_tree_set neighborhood;
    typedef combo_tree_set_it neighborhood_it;
    typedef combo_tree_set_const_it neighborhood_const_it;

public:

    NeighborhoodGenerator(const operator_set& operators,
                          const combo_tree_ns_set& perceptions,
                          const combo_tree_ns_set& actions,
                          const rule& action_reduction,
                          const rule& full_reduction,
                          arity_t arg_count = 0,
                          bool abibb = false)
    : _operators(operators), _perceptions(perceptions), _actions(actions),
    _action_reduction(action_reduction), _full_reduction(full_reduction),
    _arg_count(arg_count), _action_boolean_if_both_branches(abibb) {
        opencog::cassert(TRACE_INFO, arg_count >= 0);
    }

    ~NeighborhoodGenerator() {}

    void clearCompositeActions() {
        _composite_actions.clear();
    }

    void clearCompositePerceptions() {
        _composite_perceptions.clear();
    }

    //composite perceptions are all simple combo_tree construct of boolean type
    void precomputeCompositePerceptions() {
        for (combo_tree_ns_set_const_it i = _perceptions.begin();
                i != _perceptions.end(); ++i) {
            //add elementary perceptions
            _composite_perceptions.insert(*i);
        }
        //generate composite perceptions depending on the operator used
        for (operator_set_const_it osi = _operators.begin();
                osi != _operators.end(); ++osi) {
            //-----------
            //logical_not
            //-----------
            if (*osi == id::action_boolean_if) {
                //add logical_not(perception)
                for (combo_tree_ns_set_it i = _perceptions.begin();
                        i != _perceptions.end(); ++i) {
                    combo_tree tmp(*i);
                    tmp.wrap(tmp.begin(), id::logical_not);
                    _composite_perceptions.insert(tmp);
                }
            }
        }
    }

    //composite actions are all simple combo_tree construct of action result type
    //IMPORTANT : precomputeCompositePerceptions() must be executed prior
    //precomputeCompositeActions otherwise all composite actions involing
    //perceptions are discarded
    void precomputeCompositeActions() {
        //add elementary actions
        for (combo_tree_ns_set_const_it i = _actions.begin(); i != _actions.end(); ++i) {
            _composite_actions.insert(*i); //no use of add_composite_action
            //because there is no action reduction
        }
        //generate composite actions depending on the operator used
        for (operator_set_const_it osi = _operators.begin(); osi != _operators.end(); ++osi) {

            //-----------------
            //action_boolean_if
            //-----------------
            if (*osi == id::action_boolean_if) {
                for (combo_tree_ns_set_const_it cond = _composite_perceptions.begin();
                        cond != _composite_perceptions.end(); ++cond) {
                    opencog::cassert(TRACE_INFO, !cond->empty(),
                                     "condition cannot be empty");
                    pre_it cond_head_it = cond->begin();
                    for (combo_tree_ns_set_const_it act1 = _actions.begin();
                            act1 != _actions.end(); ++act1) {
                        opencog::cassert(TRACE_INFO, !act1->empty(),
                                         "atomic action cannot be empty");
                        pre_it act1_head_it = act1->begin();
                        //add action_boolean_if(cond and_seq(act) and_seq())
                        combo_tree tmp1;
                        tmp1.set_head(id::action_boolean_if);
                        pre_it head_it = tmp1.begin();
                        //add condition
                        tmp1.replace(tmp1.append_child(head_it), cond_head_it);
                        //add and_seq(act) at first branche
                        pre_it br1 = tmp1.append_child(head_it, id::sequential_and);
                        tmp1.replace(tmp1.append_child(br1), act1_head_it);
                        //add and_seq() at second branche
                        pre_it second_and_seq_it = tmp1.append_child(head_it,
                                                   id::sequential_and);
                        add_composite_action(tmp1);
                        //add the second branche if _conditional_both_branch_extension
                        if (_action_boolean_if_both_branches) {
                            for (combo_tree_ns_set_const_it act2 = _actions.begin();
                                    act2 != _actions.end(); ++act2) {
                                pre_it act2_head_it = act2->begin();
                                combo_tree tmp2 = tmp1;
                                pre_it sas_it2 = get_same_position(tmp1, tmp2,
                                                                   second_and_seq_it);
                                tmp2.replace(tmp2.append_child(sas_it2), act2_head_it);
                                add_composite_action(tmp2);
                            }
                        }
                    }
                }
            }
            //----------------
            //action_action_if
            //----------------
            else if (*osi == id::action_action_if) {
                for (combo_tree_ns_set_const_it act_cond = _actions.begin();
                        act_cond != _actions.end(); ++act_cond) {
                    opencog::cassert(TRACE_INFO, !act_cond->empty(),
                                     "condition action cannot be empty");
                    combo_tree tmp_cond(act_cond->begin());
                    pre_it cond_head = tmp_cond.begin();
                    cond_head = tmp_cond.wrap(cond_head, id::sequential_and);
                    for (combo_tree_ns_set_const_it act = _actions.begin();
                            act != _actions.end(); ++act) {
                        opencog::cassert(TRACE_INFO, !act->empty(),
                                         "atomic action cannot be empty");
                        pre_it act_head = act->begin();
                        { //add action_action_if(cond and_seq(act) and_seq())
                            combo_tree tmp1;
                            tmp1.set_head(id::action_action_if);
                            pre_it head = tmp1.begin();
                            //add condition
                            tmp1.replace(tmp1.append_child(head), cond_head);
                            //add and_seq(act) at first branche
                            pre_it br1 = tmp1.append_child(head, id::sequential_and);
                            tmp1.replace(tmp1.append_child(br1), act_head);
                            //add and_seq() at second branche
                            tmp1.append_child(head, id::sequential_and);
                            add_composite_action(tmp1);
                        }
                        { //add action_action_if(cond and_seq() and_seq(act))
                            combo_tree tmp2;
                            tmp2.set_head(id::action_action_if);
                            pre_it head = tmp2.begin();
                            //add condition
                            tmp2.replace(tmp2.append_child(head), cond_head);
                            //add and_seq(act) at first branche
                            tmp2.append_child(head, id::sequential_and);
                            //add and_seq() at second branche
                            pre_it br2 = tmp2.append_child(head, id::sequential_and);
                            tmp2.replace(tmp2.append_child(br2), act_head);
                            add_composite_action(tmp2);
                        }
                    }
                }
            }
            //------------
            //action_while
            //------------
            else if (*osi == id::action_while) {
                for (combo_tree_ns_set_const_it act = _actions.begin();
                        act != _actions.end(); ++act) {
                    opencog::cassert(TRACE_INFO, !act->empty(),
                                     "while action cannot be empty");
                    {
                        combo_tree tmp;
                        tmp.set_head(id::action_while);
                        pre_it tmp_head = tmp.begin();
                        pre_it tmp_as = tmp.append_child(tmp_head, id::sequential_and);
                        tmp.replace(tmp.append_child(tmp_as), pre_it(act->begin()));
                        add_composite_action(tmp);
                    }
                    //if action_not in operator set
                    //then add the same case with action_not before and_seq
                    if (_operators.find(id::action_not) != _operators.end()) {
                        combo_tree tmp;
                        tmp.set_head(id::action_while);
                        pre_it tmp_head = tmp.begin();
                        pre_it tmp_as = tmp.append_child(tmp_head, id::sequential_and);
                        tmp.replace(tmp.append_child(tmp_as), pre_it(act->begin()));
                        tmp.wrap(tmp_as, id::action_not);
                        add_composite_action(tmp);
                    }
                }
            }
            //-------------
            //boolean_while
            //-------------
            else if (*osi == id::boolean_while) {
                for (combo_tree_ns_set_const_it cond = _composite_perceptions.begin();
                        cond != _composite_perceptions.end(); ++cond) {
                    opencog::cassert(TRACE_INFO, !cond->empty(),
                                     "while condition cannot be empty");
                    for (combo_tree_ns_set_const_it act = _actions.begin();
                            act != _actions.end(); ++act) {
                        opencog::cassert(TRACE_INFO, !act->empty(),
                                         "while action cannot be empty");
                        combo_tree tmp;
                        tmp.set_head(id::boolean_while);
                        pre_it tmp_head = tmp.begin();
                        pre_it tmp_cond = tmp.append_child(tmp_head);
                        pre_it tmp_act = tmp.append_child(tmp_head, id::sequential_and);
                        tmp.replace(tmp_cond, pre_it(cond->begin()));
                        tmp.replace(tmp.append_child(tmp_act), pre_it(act->begin()));
                        add_composite_action(tmp);
                    }
                }
            }
        }
    }

    //populate the neighborhood of center, dealing with variations
    //on the subtree starting from it
    void populate_neighborhood(neighborhood& nh, const combo_tree& tr, pre_it it) {
        opencog::cassert(TRACE_INFO, !tr.empty(), "center should not be empty");
        opencog::cassert(TRACE_INFO, tr.is_valid(it),
                         "Invalide node associated to the tree");
        //proceed recursively by case
        //-------
        //and_seq
        //-------
        if (*it == id::sequential_and) {
            //remove any child of and_seq
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                combo_tree tmp = tr;
                pre_it tmp_sib = get_same_position(tr, tmp, pre_it(sib));
                tmp.erase(tmp_sib);
                add_neighbor(nh, tmp);
            }
            //insert any composite action
            for (combo_tree_ns_set_const_it compact = _composite_actions.begin();
                    compact != _composite_actions.end(); ++compact) {
                opencog::cassert(TRACE_INFO, !compact->empty(),
                                 "composite action cannot be empty");
                pre_it compact_head = compact->begin();
                { //insert first child composite action
                    combo_tree tmp = tr;
                    pre_it tmp_it = get_same_position(tr, tmp, it);
                    tmp.replace(tmp.prepend_child(tmp_it), compact_head);
                    add_neighbor(nh, tmp);
                }
                //for each child insert after composite action
                for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                    combo_tree tmp = tr;
                    pre_it tmp_sib = get_same_position(tr, tmp, pre_it(sib));
                    tmp.insert_subtree_after(tmp_sib, compact_head);
                    add_neighbor(nh, tmp);
                }
            }
            //apply recursively that population method on the children
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                populate_neighborhood(nh, tr, pre_it(sib));
        }
        //-----------------
        //action_boolean_if
        //-----------------
        else if (*it == id::action_boolean_if) {
            opencog::cassert(TRACE_INFO, it.number_of_children() == 3,
                             "action_boolean_if has necessarily 3 children");
            //apply recursively that population method
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 0)));
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 1)));
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 2)));
        }
        //----------------
        //action_action_if
        //----------------
        else if (*it == id::action_action_if) {
            opencog::cassert(TRACE_INFO, it.number_of_children() == 3,
                             "action_boolean_if has necessarily 3 children");
            //apply recursively that population method
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 0)));
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 1)));
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 2)));
            //with inverse branche 1 and branche 2 then apply recursively on cond
            {
                combo_tree tmp = tr;
                pre_it tmp_it = get_same_position(tr, tmp, it);
                pre_it tmp_cond = tmp_it.begin();
                pre_it tmp_br1 = tmp.child(tmp_it, 1);
                pre_it tmp_br2 = tmp.child(tmp_it, 2);
                tmp.swap(tmp_br1, tmp_br2);
                populate_neighborhood(nh, tmp, tmp_cond);
            }
            //transform the action_action_if in sequential_and by
            //concatenating the condition action and one of the branch
            { //concatenation with branch 1
                combo_tree tmp1 = tr;
                pre_it tmp1_it = get_same_position(tr, tmp1, it);
                *tmp1_it = id::sequential_and;
                tmp1.erase(tmp1_it.last_child());
                add_neighbor(nh, tmp1);
            }
            { //concatenation with branch 2
                combo_tree tmp2 = tr;
                pre_it tmp2_it = get_same_position(tr, tmp2, it);
                *tmp2_it = id::sequential_and;
                tmp2.erase(tmp2.child(tmp2_it, 1));
                add_neighbor(nh, tmp2);
            }
        }
        //------------
        //action_while
        //------------
        else if (*it == id::action_while) {
            opencog::cassert(TRACE_INFO, it.has_one_child(),
                             "action_while has necessarily one child");
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 0)));
            //if action_not is in the operator set then
            //wrap action_not on action_while child
            if (_operators.find(id::action_not) != _operators.end()) {
                combo_tree tmp = tr;
                pre_it tmp_it = get_same_position(tr, tmp, it);
                tmp.wrap(tmp_it, id::action_not);
                add_neighbor(nh, tmp);
            }
        }
        //-------------
        //boolean_while
        //-------------
        else if (*it == id::boolean_while) {
            opencog::cassert(TRACE_INFO, it.number_of_children() == 2,
                             "boolean_while has necessarily 2 children");
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 0)));
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 1)));
        }
        //----------
        //action_not
        //----------
        else if (*it == id::action_not) {
            opencog::cassert(TRACE_INFO, it.has_one_child(),
                             "action_not must have one child");
            populate_neighborhood(nh, tr, pre_it(it.begin()));
            //if action_not is in the operator set then
            //delete action_not
            if (_operators.find(id::action_not) != _operators.end()) {
                combo_tree tmp = tr;
                pre_it tmp_it = get_same_position(tr, tmp, it);
                tmp.erase(tmp.flatten(tmp_it));
                add_neighbor(nh, tmp);
            }
        }
        //-----------
        //logical_not
        //-----------
        else if (*it == id::logical_not) {
            opencog::cassert(TRACE_INFO, it.has_one_child(),
                             "logical_not must have one child");
            populate_neighborhood(nh, tr, pre_it(tr.child(it, 0)));
        }
        //----------
        //perception
        //----------
        else if (is_perception(*it)) {
            for (combo_tree_ns_set_const_it cond = _composite_perceptions.begin();
                    cond != _composite_perceptions.end(); ++cond) {
                opencog::cassert(TRACE_INFO, !cond->empty(), "condition cannot be empty");
                pre_it cond_head = cond->begin();
                combo_tree tmp = tr;
                pre_it tmp_it = get_same_position(tr, tmp, it);
                tmp.erase_children(tmp_it);
                tmp.replace(tmp_it, cond_head);
                add_neighbor(nh, tmp);
            }
        }
        //--------------
        //builtin action
        //--------------
        else if (is_builtin_action(*it)) {
            //substitute the action by any composite actions
            for (combo_tree_ns_set_const_it compact = _composite_actions.begin();
                    compact != _composite_actions.end(); ++compact) {
                opencog::cassert(TRACE_INFO, !compact->empty(),
                                 "composite action cannot be empty");
                pre_it compact_head = compact->begin();
                combo_tree tmp = tr;
                pre_it tmp_it = get_same_position(tr, tmp, it);
                tmp.erase_children(tmp_it);
                tmp.replace(tmp_it, compact_head);
                add_neighbor(nh, tmp);
            }
#ifdef PERMUTATION_NEIGHBORHOOD_EXPANSION
            //move the builtin action under other sequential_and
            {
                combo_tree tmp = tr;
                pre_it tmp_it = get_same_position(tr, tmp, it);
                combo_tree act_tr(tmp_it);
                tmp.erase(tmp_it);
                for (pre_it pos_it = tmp.begin(); pos_it != tmp.end(); ++pos_it) {
                    if (*pos_it == id::sequential_and) {
                        //prepend child act_tr
                        {
                            combo_tree ins_tmp = tmp;
                            pre_it ins_tmp_it = get_same_position(tmp, ins_tmp, pos_it);
                            ins_tmp.replace(ins_tmp.prepend_child(ins_tmp_it),
                                            act_tr.begin());
                            add_neighbor(nh, ins_tmp);
                        }
                        //insert after each child
                        for (sib_it sib = pos_it.begin(); sib != pos_it.end(); ++sib) {
                            combo_tree ins_tmp = tmp;
                            pre_it ins_tmp_it = get_same_position(tmp, ins_tmp,
                                                                  pre_it(sib));
                            ins_tmp.insert_subtree_after(ins_tmp_it, act_tr.begin());
                            add_neighbor(nh, ins_tmp);
                        }
                    }
                }
            }
#endif
        }
        //--------------------------------------------------
        //no case, should not get into that part of the code
        //--------------------------------------------------
        else opencog::cassert(TRACE_INFO, false,
                                  "Should not get into that part of the code");
    }

    const combo_tree_ns_set& getCompositePerceptions() const {
        return _composite_perceptions;
    }

    const combo_tree_ns_set& getCompositeActions() const {
        return _composite_actions;
    }

private:

    //private attrivutes

    const operator_set& _operators;
    const combo_tree_ns_set& _perceptions;
    const combo_tree_ns_set& _actions;

    const rule& _action_reduction;
    const rule& _full_reduction;

    arity_t _arg_count; //number of input arguments a neighbor must have

    //if that flag is true then the neighborhood expansion
    //consider filling 2 branches of a conditional in one step
    bool _action_boolean_if_both_branches;

    combo_tree_ns_set _composite_perceptions;
    combo_tree_ns_set _composite_actions;

    //private methods

    //add the combo_tree in the neighborhood
    void add_neighbor(neighborhood& nh, combo_tree& tr) {
#ifdef APPLY_FULL_REDUCTION
        _full_reduction(tr);
#endif
        //debug log
        std::stringstream ss_tr;
        ss_tr << tr;
        std::string s_tr = ss_tr.str();
        logger().log(opencog::Logger::DEBUG,
                     "NeighborhoodGenerator - Insert neighbor : %s",
                     s_tr.c_str());
        //~debug log

        //insert only if tr contains exactly arg_count input arguments
        if (does_contain_all_arg_up_to(tr, _arg_count))
            nh.insert(tr);
    }

    //add composite action
    void add_composite_action(combo_tree& tr) {
        bool insert_action = true;
#ifdef APPLY_ACTION_REDUCTION
        _action_reduction(tr);
        //check if the composite action is of the form and_seq(builtin_action)
        //if it is then do not insert it since it would already have been
        //inserted as atomic action
        opencog::cassert(TRACE_INFO, !tr.empty(), "the composite action cannot be empty");
        pre_it head = tr.begin();
        insert_action = !(//single action wrapped with and and_seq
                            (*head == id::sequential_and
                             && head.has_one_child()
                             && is_builtin_action(*head.begin()))
                            || //anything starting with action_not is not allowed
                            (*head == id::action_not)
                        );
#endif
        if (insert_action)
            _composite_actions.insert(tr);
    }

    //Not sure if this procedure is already coded in tree.h but didn't find it
    //it take the tree tr with iterator it and return the iterator that
    //would have the same position in tmp
    //NOTE : COULD BE NICE TO OPTIMZE IT IF POSSIBLE
    pre_it get_same_position(const combo_tree& tr, const combo_tree& tmp, pre_it it) {
        opencog::cassert(TRACE_INFO, !tr.empty());
        opencog::cassert(TRACE_INFO, !tmp.empty());
        opencog::cassert(TRACE_INFO, tr == tmp);
        pre_it tr_it = tr.begin();
        pre_it res = tmp.begin();
        for (; tr_it != it && tr_it != tr.end(); ++tr_it)
            ++res;
        return res;
    }

};
}

#endif
