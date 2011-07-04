/*
 * opencog/comboreduct/reduct/action_rules.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#include "action_rules.h"

#include <opencog/util/exceptions.h>
#include <opencog/comboreduct/combo/descriptions.h>

namespace opencog { namespace reduct {

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

//for ome intriging reason the operator tribool.safe_bool() is not usable
//because considered private by gcc?
//so I code one here
bool safe_bool(boost::tribool t)
{
    //if(boost::indeterminate(t))
    //  return false;
    //else if(t)
    //  return true;
    //else return false;
    return (bool)t;
}

//takes a combo subtree of output type action_result starting at 'it'
//and compute whether that combo subtree
//1) always succeeds (in that case it returns true),
//2) always fails (in that case it returns false),
//3) sometimes succeeds or fails (in this case it returns unknown)
boost::tribool get_action_result(const combo_tree& tr, pre_it it)
{
    OC_ASSERT(!tr.empty(), "combo_tree cannot be empty");
    OC_ASSERT(tr.is_valid(it),
                     "the iterator must be valid");
    //base cases
    if (*it == id::action_success)
        return true;
    else if (*it == id::action_failure)
        return false;
    else if (*it == id::sequential_exec)
        return true;
    else if (*it == id::action_while)
        return false;
    else if (is_builtin_action(*it)) {
        if (get_builtin_action(*it)->always_succeeds())
            return true;
        else return boost::logic::indeterminate;
    }
    //recursive cases
    else if (*it == id::sequential_and) {
        boost::tribool res = true;
        for (sib_it sib = it.begin(); sib != it.end() && safe_bool(res); ++sib) {
            res = get_action_result(tr, pre_it(sib));
        }
        return res;
    } else if (*it == id::sequential_or) {
        boost::tribool res = false;
        bool exists_indeterminate = false;
        for (sib_it sib = it.begin(); sib != it.end() && !safe_bool(res); ++sib) {
            res = get_action_result(tr, pre_it(sib));
            exists_indeterminate = boost::indeterminate(res)
                                   || exists_indeterminate;
        }
        if (safe_bool(res))
            return true;
        else if (exists_indeterminate)
            return boost::logic::indeterminate;
        else return false;
    } else if (*it == id::action_not) {
        OC_ASSERT(it.has_one_child(),
                         "action must have one child");
        return !get_action_result(tr, pre_it(it.begin()));
    } else if (*it == id::action_action_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "must have 3 children");
        pre_it cond = tr.child(it, 0);
        pre_it br1 = tr.child(it, 1);
        pre_it br2 = tr.child(it, 2);
        boost::tribool cond_result = get_action_result(tr, cond);
        if (cond_result)
            return get_action_result(tr, br1);
        else if (!cond_result)
            return get_action_result(tr, br2);
        else return boost::logic::indeterminate;
    }
    //default case
    else return boost::logic::indeterminate;
}

//replace all cond within tr (for 'it') occuring at the initial instant
void substitute_condition_init_instant(combo_tree& tr, pre_it it,
                                       pre_it cond, vertex sub)
{
    OC_ASSERT(!tr.empty(), "tr must not be empty");
    OC_ASSERT(tr.is_valid(it) && tr.is_valid(cond),
                     "it and cond must be valid");
    vertex v = *it;
    //base cases
    if (is_builtin_action(v) || v == id::action_success || v == id::action_failure)
        return;
    else if (is_perception(v)) {
        if (tr.equal_subtree(it, cond)) {
            tr.erase_children(it);
            *it = sub;
        }
    }
    //recursive cases
    else if (v == id::sequential_and ||
             v == id::sequential_or ||
             v == id::sequential_exec) {
        if (!it.is_childless()) {
            //only the first child because the other will not correspond to
            //the same instant
            substitute_condition_init_instant(tr, pre_it(it.begin()), cond, sub);
        }
    } else if (v == id::action_boolean_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "action_boolean_if must have 3 children");
        //call recursively on the 3 children because they all start at the same
        //instant
        substitute_condition_init_instant(tr, pre_it(tr.child(it, 0)), cond, sub);
        substitute_condition_init_instant(tr, pre_it(tr.child(it, 1)), cond, sub);
        substitute_condition_init_instant(tr, pre_it(tr.child(it, 2)), cond, sub);
    } else if (v == id::action_action_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "action_action_if must have 3 children");
        //recursive call only on the first child because then the other
        //branche start later
        substitute_condition_init_instant(tr, pre_it(tr.child(it, 0)), cond, sub);
    } else return;
}

//--------------------------------------------------------------------------
//General action rules
//--------------------------------------------------------------------------

//action_boolean_if(true A B) -> A
//action_boolean_if(false A B) -> B
//action_boolean_if(C A A) -> A
void reduce_action_if::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::action_boolean_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "combo_tree node should have exactly three children (reduce_action_if)");
        pre_it cond = tr.child(it, 0);
        pre_it b1 = tr.child(it, 1);
        pre_it b2 = tr.child(it, 2);
        if (*cond == id::logical_true) {
            *it = *b1;
            tr.erase(cond);
            tr.erase(b2);
            tr.erase(tr.flatten(b1));
        } else if (*cond == id::logical_false) {
            *it = *b2;
            tr.erase(cond);
            tr.erase(b1);
            tr.erase(tr.flatten(b2));
        } else if (tr.equal_subtree(b1, b2)) {
            *it = *b1;
            tr.erase(cond);
            tr.erase(b2);
            tr.erase(tr.flatten(b1));
        }
    }
}


//action_action_if(A B B) -> and_seq(exec_seq(A) B)
void reduce_action_action_if::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::action_action_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "combo_tree node should have exactly three children (reduce_action_action_if)");
        pre_it cond = tr.child(it, 0);
        pre_it b1 = tr.child(it, 1);
        pre_it b2 = tr.child(it, 2);

        if (tr.equal_subtree(b1, b2)) {
            *it = id::sequential_and;
            tr.erase(b2);
            tr.wrap(cond, id::sequential_exec);
        }
    }
}

//T_action_if(action_failure A B) -> B
//T_action_if(action_success A B) -> A
//with T in {action, boolean, contin}
void reduce_const_cond_action_if::operator()
(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::action_action_if ||
            *it == id::boolean_action_if ||
            *it == id::contin_action_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "combo_tree node should have exactly three children (reduce_cond_cond_action_if)");
        pre_it cond = tr.child(it, 0);
        pre_it b1 = tr.child(it, 1);
        pre_it b2 = tr.child(it, 2);
        if (*cond == id::action_failure) {
            *it = *b2;
            tr.erase(cond);
            tr.erase(b1);
            tr.erase(tr.flatten(b2));
        } else if (*cond == id::action_success) {
            *it = *b1;
            tr.erase(cond);
            tr.erase(b2);
            tr.erase(tr.flatten(b1));
        }
    }
}

//action_boolean_if(not(A) B C) -> action_boolean_if(A C B)
void reduce_not_cond_action_boolean_if::operator()
(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::action_boolean_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "action_boolean_if must have 3 children");
        pre_it cond = it.begin();
        if (*cond == id::logical_not) {
            tr.erase(tr.flatten(cond));
            tr.swap(tr.child(it, 1));
        }
    }
}


//and_seq(A action_failure B) -> and_seq(A action_failure)
//and_seq(action_failure) -> action_failure
//and_seq(A action_success B) -> and_seq(A B)
//or_seq(A action_failure B) -> or_seq(A B)
//or_seq(A action_success B) -> or_seq(A action_success)
//or_seq(action_success) -> action_success
//exec_seq(A action_failure B) -> exec_seq(A B)
//exec_seq(A action_success C) -> exec_seq(A B)
void reduce_const_action_seq::operator()
(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::sequential_and) {
        if (it.has_one_child() && *it.begin() == id::action_failure) {
            tr.erase_children(it);
            *it = id::action_failure;
        } else {
            bool erase = false;
            for (sib_it sib = it.begin(); sib != it.end();) {
                if (*sib == id::action_failure) {
                    erase = true;
                    ++sib;
                } else if (erase || *sib == id::action_success)
                    sib = tr.erase(sib);
                else ++sib;
            }
        }
    } else if (*it == id::sequential_or) {
        if (it.has_one_child() && *it.begin() == id::action_success) {
            tr.erase_children(it);
            *it = id::action_success;
        } else {
            bool erase = false;
            for (sib_it sib = it.begin(); sib != it.end();) {
                if (*sib == id::action_success) {
                    erase = true;
                    ++sib;
                } else if (erase || *sib == id::action_failure)
                    sib = tr.erase(sib);
                else ++sib;
            }
        }
    } else if (*it == id::sequential_exec) {
        for (sib_it sib = it.begin(); sib != it.end();) {
            if (*sib == id::action_failure || *sib == id::action_success)
                sib = tr.erase(sib);
            else ++sib;
        }
    }
}

//and_seq() -> action_success
//or_seq() -> action_failure
//exec_seq() -> action_success
void reduce_empty_arg_seq::operator() (combo_tree& tr, pre_it it) const
{
    if (it.is_childless()) {
        if (*it == id::sequential_and || *it == id::sequential_exec)
            *it = id::action_success;
        else if (*it == id::sequential_or)
            *it = id::action_failure;
    }
}

//action_not(action_not(A)) -> A
void reduce_double_action_not::operator() (combo_tree& tr, pre_it it) const
{
    if (*it == id::action_not) {
        OC_ASSERT(it.has_one_child(),
                         "action_not must have one child");
        pre_it it_child = it.begin();
        if (*it_child == id::action_not) {
            tr.erase(tr.flatten(it_child));
            *it = *it.begin();
            tr.erase(tr.flatten(it.begin()));
        }
    }
}

//and_seq(A B action_while(B)) -> and_seq(A action_while(B))
void reduce_repeat_out_action_while::operator() (combo_tree& tr, pre_it it) const
{
    if (*it == id::sequential_and) {
        if (!it.has_one_child()) { //if and_seq has only one child then the rule
            //will not do anything anyway
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                if (*sib == id::action_while) {
                    OC_ASSERT(sib.has_one_child(),
                                     "action_while has only one child");
                    pre_it while_child = sib.begin();
                    if (!while_child.is_childless()) {
                        //if the argument of action_while is a sequential_and then
                        //check if and_seq(preceding siblings) is equal to it
                        if (*while_child == id::sequential_and) {
                            sib_it out_sib = sib; //out of the action_while
                            sib_it in_sib = while_child.end(); //in the action_while
                            bool match_so_far = true;
                            do {
                                --out_sib;
                                --in_sib;
                                if (tr.is_valid(out_sib))
                                    match_so_far = tr.equal_subtree(in_sib, out_sib);
                                else match_so_far = false;
                            } while (in_sib != while_child.begin() && match_so_far);
                            //if all match then erase the out part
                            if (match_so_far) {
                                while (out_sib != sib) {
                                    out_sib = tr.erase(out_sib);
                                }
                            }
                        } else {
                            //if the argument of action_while is not a sequential_and then
                            //check if preceding sibling (if there is so) is equal to it
                            pre_it prev_it = tr.previous_sibling(sib);
                            if (tr.is_valid(prev_it) &&
                                    tr.equal_subtree(prev_it, while_child)) {
                                tr.erase(prev_it);
                            }
                        }
                    }
                }
            }
        }
    }
}

//action_while(and_seq(A A)) -> action_while(A)
//more generally action_while(and_seq(A^n)) -> action_while(A)
void reduce_repeat_in_action_while::operator() (combo_tree& tr, pre_it it) const
{
    if (*it == id::action_while) {
        OC_ASSERT(it.has_one_child(),
                         "action_while must have one child");
        pre_it while_child = it.begin();
        if (*while_child == id::sequential_and) {
            if (while_child.number_of_children() > 1) {
                int seq_size = while_child.number_of_children();
                int sd = smallest_divisor(seq_size);
                //get the start and the end of the pattern
                sib_it pattern_from = while_child.begin();
                sib_it pattern_to = pattern_from;
                for (int i = 0; i < sd; ++i)
                    ++pattern_to;
                sib_it pattern_to_cpy = pattern_to;
                //compare that pattern to the rest of the action sequence
                bool correct_pattern_so_far = true;
                for (; pattern_to != while_child.end() && correct_pattern_so_far;
                        ++pattern_from, ++pattern_to) {
                    correct_pattern_so_far = tr.equal_subtree(pattern_from, pattern_to);
                }
                //if the pattern was repeated all way then delete all repeated copy
                if (correct_pattern_so_far) {
                    for (sib_it sib = pattern_to_cpy; sib != while_child.end();) {
                        sib = tr.erase(sib);
                    }
                }
            }
        }
    }
}

//action_boolean_if(C A1 A2) -> action_boolean_if(C A1[C->true] A2[C->false])
//wherever A1 and A2 has no side effect, or (to rephrase) when evaluation
//of C within A1 or A2 occurs at the same instant as C within
//action_boolean_if condition
void reduce_action_boolean_if_sub_cond::operator() (combo_tree& tr,
        pre_it it) const
{
    if (*it == id::action_boolean_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "The number of children of action_boolean_if must be 3");
        //substitute in subtre of branche 1 and branche 2
        pre_it cond = it.begin();
        substitute_condition_init_instant(tr, pre_it(tr.child(it, 1)),
                                          cond, id::logical_true);
        substitute_condition_init_instant(tr, pre_it(tr.child(it, 2)),
                                          cond, id::logical_false);
    }
}


//boolean_while(C A) -> boolean_while(C A[C->true])
//wherever A has no side effect, or to rephrase,
//when evaluation of C within A occurs at the same instant as
//C within boolean_while condition.
void reduce_boolean_while_sub_cond::operator() (combo_tree& tr, pre_it it) const
{
    if (*it == id::boolean_while) {
        OC_ASSERT(it.number_of_children() == 2,
                         "The number of children of boolean_while must be 3");
        //substitute in subtree of branche 1
        pre_it cond = it.begin();
        substitute_condition_init_instant(tr, pre_it(tr.child(it, 1)),
                                          cond, id::logical_true);
    }
}

//--------------------------------------------------------------------------
//Action reduction rules based on always_succeeds properties
//--------------------------------------------------------------------------

//action_action_if(A B C) -> and_seq(A B) iff get_action_result(A)==true
void reduce_action_action_if_always_succeeds::operator() (combo_tree& tr,
        pre_it it) const
{
    if (*it == id::action_action_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "The number of children of action_action_if mut be 3");
        //check if the action at the conditions always succeeds
        if (safe_bool(get_action_result(tr, pre_it(it.begin())))) {
            *it = id::sequential_and;
            tr.erase(it.last_child()); //erase branch 2
        }
    }
}

//action_action_if(A B C) -> exec_seq(A C) iff get_action_result(A)==false
void reduce_action_action_if_always_fails::operator() (combo_tree& tr,
        pre_it it) const
{
    if (*it == id::action_action_if) {
        OC_ASSERT(it.number_of_children() == 3,
                         "The number of children of action_action_if mut be 3");
        //check if the action at the conditions always succeeds
        if (!safe_bool(get_action_result(tr, pre_it(it.begin())))) {
            *it = id::sequential_exec;
            tr.erase(tr.child(it, 1)); //erase branch 1
        }
    }
}

//action_while(A) -> A iff get_action_result(A)==false
void reduce_action_while_always_fails::operator() (combo_tree& tr,
        pre_it it) const
{
    if (*it == id::action_while) {
        OC_ASSERT(it.has_one_child(), "action_while has only 1 child");
        //check if the action at the conditions always fails
        if (safe_bool(!get_action_result(tr, pre_it(it.begin())))) {
            //remove action_while operator
            *it = *it.begin();
            tr.erase(tr.flatten(it.begin()));
        }
    }
}

//boolean_while(C A) -> action_sucess iff eval(C)==false
//boolean_while(C A) -> A iff eval(C)==true and get_action_result(A)==false
void reduce_boolean_while_depend_condition::operator() (combo_tree& tr,
        pre_it it) const
{
    if (*it == id::boolean_while) {
        OC_ASSERT(it.number_of_children() == 2,
                         "boolean_while must have 2 children");
        pre_it cond = it.begin();
        //check if the condition is true
        if (*cond == id::logical_true) {
            //erase condition
            tr.erase(cond);
            //remove boolean_while operator
            *it = *it.begin();
            tr.erase(tr.flatten(it.begin()));
        }
        //check if the condition is false
        else if (*cond == id::logical_false) {
            tr.erase_children(it);
            *it = id::action_success;
        }
    }
}

//and_seq(A1 ... An ... Am) -> and_seq(A1 ... An)
//iff exists n and for all i<n, get_action_result(Ai) = true or undetermined
//and get_action_result(An)==false
void reduce_sequential_and_always_fails::operator() (combo_tree& tr,
        pre_it it) const
{
    if (*it == id::sequential_and) {
        bool erase_mode = false;
        for (sib_it sib = it.begin(); sib != it.end();) {
            if (erase_mode) {
                sib = tr.erase(sib);
            } else {
                boost::tribool res = get_action_result(tr, pre_it(sib));
                if (safe_bool(!res))
                    erase_mode = true;
                ++sib;
            }
        }
    }
}

//or_seq(A1 ... An ... Am) -> or_seq(A1 ... An)
//iff exists n and for all i<n, get_action_result(Ai) = false or undetermined
//and get_action_result(An)==true
void reduce_sequential_or_always_succeeds::operator() (combo_tree& tr,
        pre_it it) const
{
    if (*it == id::sequential_or) {
        bool erase_mode = false;
        for (sib_it sib = it.begin(); sib != it.end();) {
            if (erase_mode) {
                sib = tr.erase(sib);
            } else {
                boost::tribool res = get_action_result(tr, pre_it(sib));
                if (safe_bool(res))
                    erase_mode = true;
                ++sib;
            }
        }
    }
}

//--------------------------------------------------------------------------
//Action reduction rules depending on properties
//--------------------------------------------------------------------------

//reduce 2 consecutive actions to 1 action
//if they have agurments then the arguments must be equal
void reduce_idempotent::operator() (combo_tree& tr, combo_tree::iterator it) const
{
    if (is_builtin_action(*it) && get_builtin_action(*it)->is_idempotent()) {
        pre_it ns_it = tr.next_sibling(it);
        if (tr.is_valid(ns_it) && tr.equal_subtree(it, ns_it))
            tr.erase(ns_it);
    }
}

//reduce 2 consecutive opposite actions A1 A2
//that is A1 is the reversal of A2
void reduce_opposite::operator() (combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::sequential_exec) {
        //note it is important *it to not be sequential_or/and because
        //in that case eliminating actions could change the semantics
        for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
            if (is_builtin_action(*sib)) {
                builtin_action a = get_builtin_action(*sib);
                if (a->is_reversible()) {
                    sib_it next_sib = tr.next_sibling(sib);
                    if (tr.is_valid(next_sib) && a->get_reversal() == *next_sib) {
                        sib = tr.erase(sib);
                        sib = tr.erase(sib);
                    }
                }
            }
        }
    }
}

//reduce 2 consecutive actions to 1 action and add the arguments that
//are denoted additive by is_additive(action,int i);
//the argument that are not denoted additive must be equal
void reduce_additive::operator() (combo_tree& tr, combo_tree::iterator it) const
{
    if (is_builtin_action(*it)) {
        builtin_action a = get_builtin_action(*it);
        if (a->exists_additive_argument()) {
            pre_it ns_it = tr.next_sibling(it);
            if (tr.is_valid(ns_it) && *it == *ns_it) {
                OC_ASSERT(tr.number_of_children(it) == tr.number_of_children(ns_it),
                                 "combo_tree node and its next sibling should have the same number of children.");
                std::vector<int> arg_add; //vector of additive arguments
                for (unsigned int i = 0; i < tr.number_of_children(it); ++i) {
                    if (a->is_additive(i))
                        arg_add.push_back(i);
                    else if (!tr.equal_subtree(tr.child(it, i), tr.child(ns_it, i)))
                        return;
                }
                for (std::vector<int>::const_iterator arg_i = arg_add.begin();
                        arg_i != arg_add.end(); ++arg_i) {
                    pre_it it_child = tr.child(it, *arg_i);
                    pre_it ns_it_child = tr.child(ns_it, *arg_i);
                    if (is_contin(*it_child) && is_contin(*ns_it_child))
                        *it_child = get_contin(*it_child) + get_contin(*ns_it_child);
                    else {
                        tr.wrap(it_child, id::plus);
                        tr.move_after(it_child, ns_it_child);
                    }
                }
                tr.erase(ns_it);
            }
        }
    }
}

//replace action by action_success if exists 0 in one of its argument
//neutral
void reduce_zero_neutral::operator() (combo_tree& tr, combo_tree::iterator it) const
{
    if (is_builtin_action(*it)) {
        builtin_action a = get_builtin_action(*it);
        if (a->exists_zero_neutral_argument()) {
            bool is_neutral = false;
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                if (a->is_zero_neutral(tr.sibling_index(sib))
                        && is_contin(*sib) && get_contin(*sib) == 0.0) {
                    is_neutral = true;
                    break;
                }
            }
            if (is_neutral) {
                *it = id::action_success;
                tr.erase_children(it);
            }
        }
    }
}


//action_with_modular_argument(m) -> action_with_modular_argument(reduced(m))
//for example, dummy_modular_action(6.28)-> dummy_modular_action(0)
void reduce_modular_argument::operator() (combo_tree& tr, combo_tree::iterator it) const
{
    if (is_builtin_action(*it)) {
        builtin_action a = get_builtin_action(*it);
        for (unsigned int i = 0;i < it.number_of_children();++i) {
            if (a->is_modulo(i)) {
                double min = a->modulo_min(i);
                double max = a->modulo_max(i);
                OC_ASSERT(max >= min, "max smaller than min (reduce_modular_argument)");
                pre_it it_child = tr.child(it, i);
                if (min == max)
                    *it_child = min;
                else if (get_contin(*it_child) < min)
                    *it_child = get_contin(*it_child) - (max - min) * ((int)((get_contin(*it_child) - min) / (max - min))) + (max - min);
                else if (get_contin(*it_child) >= max)
                    *it_child = get_contin(*it_child) - (max - min) * ((int)((get_contin(*it_child) - min) / (max - min)));
            }
        }
    }
}

// preconditions checks
void preconditions_check::operator() (combo_tree& tr, combo_tree::iterator it) const
{
    //NOTE TO PREDRAG :
    //change the code so that it uses builtin_action::precondition() instead
    /*
    using namespace builtin_action_properties;
    builtin_action_precedence p;
    unsigned int i=0;
    while (builtin_actions_properties::instance().builtin_action_pos_prec(i++, &p)) {
      builtin_action pre_act =p.a1;
      builtin_action post_act=p.a2;
      reduce_free_post_action (pre_act, post_act, false, tr, it);
      }*/
}

// auxilary function for preconditions checks
bool reduce_free_post_action (builtin_action pre_a, builtin_action post_a, bool free_pre_action_before, combo_tree& tr, combo_tree::iterator it)
{
    if (is_builtin_action(*it)) {
        if (pre_a == get_builtin_action(*it))
            return true;
        else
            if (post_a == get_builtin_action(*it)) {
                if (!free_pre_action_before)
                    *it = id::action_success;
                return false;
            }
    } else {
        if (!is_action(*it))
            return false;

        if (it.number_of_children() == 0)
            return false;

        bool free_pre_action = free_pre_action_before;
        action aa = get_action(*it);

        switch (aa) {
        case id::action_success:
        case id::action_failure:
        case id::return_success:
        case id::sequential_and:
        case id::sequential_exec:
        case id::action_while:
        case id::boolean_while:
        case id::repeat_n:

            for (unsigned int i = 0;i < it.number_of_children();++i) {
                pre_it child_it = tr.child(it, i);
                free_pre_action |= reduce_free_post_action (pre_a, post_a, free_pre_action, tr, child_it);
            }
            break;

        case id::sequential_or:
        case id::action_if:
            //case id::action_boolean_if:  the same as action_if
        case id::boolean_action_if:
        case id::contin_action_if:
        case id::action_action_if:

            free_pre_action = false;
            for (unsigned int i = 0;i < it.number_of_children();++i) {
                pre_it child_it = tr.child(it, i);
                free_pre_action |= reduce_free_post_action (pre_a, post_a, free_pre_action_before, tr, child_it);
            }
            break;

        default:
            break;
        }
        return free_pre_action;
    }
    return false;
}

} // ~namespace reduct
} // ~namespace opencog 
