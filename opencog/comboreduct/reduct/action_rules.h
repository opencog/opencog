/*
 * opencog/comboreduct/reduct/action_rules.h
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
#ifndef _REDUCT_ACTION_RULES_H
#define _REDUCT_ACTION_RULES_H

#include "reduct.h"
#include <boost/logic/tribool.hpp>

namespace opencog { namespace reduct {

using namespace combo;

//for some intriguing reason the operator tribool.safe_bool() is not usable
//because considered private by gcc?
//so I code one here
bool safe_bool(boost::tribool t);

//takes a combo subtree of output type action_result starting at 'it'
//and compute whether that combo subtree
//1) always succeeds (in that case it returns true),
//2) always fails (in that case it returns false),
//3) sometimes succeeds or fails (in this case it returns unknown)
boost::tribool get_action_result(const combo_tree& tr, combo_tree::iterator it);

//replace all cond within tr (for 'it') occuring at the initial instant
void substitute_condition_init_instant(combo_tree& tr, combo_tree::iterator it,
                                       combo_tree::iterator cond, vertex sub);


//--------------------------------------------------------------------------
//General action reduction rules
//--------------------------------------------------------------------------

//action_boolean_if(true A B) -> A
//action_boolean_if(false A B) -> B
//action_boolean_if(C A A) -> A
struct reduce_action_if : public crule<reduce_action_if> {
    reduce_action_if() : 
          crule<reduce_action_if>::crule("reduce_action_if") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//action_action_if(A B B) -> and_seq(exec_seq(A) B)
struct reduce_action_action_if : public crule<reduce_action_action_if> {
    reduce_action_action_if() : 
        crule<reduce_action_action_if>::crule("reduce_action_action_if") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//T_action_if(action_failure A B) -> B
//T_action_if(action_success A B) -> A
//with T in {action, boolean, contin}
struct reduce_const_cond_action_if: public crule<reduce_const_cond_action_if> {
    reduce_const_cond_action_if() :
        crule<reduce_const_cond_action_if>::crule("reduce_const_cond_action_if") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//action_boolean_if(not(A) B C) -> action_boolean_if(A C B)
struct reduce_not_cond_action_boolean_if:
        public crule<reduce_not_cond_action_boolean_if> {
    reduce_not_cond_action_boolean_if() :
        crule<reduce_not_cond_action_boolean_if>::crule("reduce_not_cond_action_boolean_if") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//and_seq(A action_failure B) -> and_seq(A action_failure)
//and_seq(A action_success B) -> and_seq(A B)
//or_seq(A action_failure B) -> or_seq(A B)
//or_seq(A action_success B) -> or_seq(A action_success)
//exec_seq(A action_failure B) -> exec_seq(A B)
//exec_seq(A action_success C) -> exec_seq(A B)
struct reduce_const_action_seq : public crule<reduce_const_action_seq> {
    reduce_const_action_seq () :
        crule<reduce_const_action_seq>::crule("reduce_const_action_seq") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//and_seq() -> action_success
//or_seq() -> action_failure
//exec_seq() -> action_success
struct reduce_empty_arg_seq : public crule<reduce_empty_arg_seq> {
    reduce_empty_arg_seq () :
        crule<reduce_empty_arg_seq>::crule("reduce_empty_arg_seq") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//action_not(action_not(A)) -> A
struct reduce_double_action_not : public crule<reduce_double_action_not> {
    reduce_double_action_not () :
        crule<reduce_double_action_not>::crule("reduce_double_action_not") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};


//and_seq(A B action_while(B)) -> and_seq(A action_while(B))
struct reduce_repeat_out_action_while : public crule<reduce_repeat_out_action_while> {
    reduce_repeat_out_action_while () :
        crule<reduce_repeat_out_action_while>::crule("reduce_repeat_out_action_while") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//action_while(and_seq(A A)) -> action_while(A)
struct reduce_repeat_in_action_while : public
            crule<reduce_repeat_in_action_while> {
    reduce_repeat_in_action_while() : crule<reduce_repeat_in_action_while>::crule("reduce_repeat_in_action_while") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//action_boolean_if(C A1 A2) -> action_boolean_if(C A1[C->true] A2[C->false])
//wherever A1 and A2 has no side effect, or (to rephrase) when evaluation
//of C within A1 or A2 occurs at the same instant as C within
//action_boolean_if condition
struct reduce_action_boolean_if_sub_cond : public
            crule<reduce_action_boolean_if_sub_cond> {
    reduce_action_boolean_if_sub_cond() : crule<reduce_action_boolean_if_sub_cond>::crule("reduce_action_boolean_if_sub_cond") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//boolean_while(C A) -> boolean_while(C A[C->true])
//wherever A has no side effect, or to rephrase,
//when evaluation of C within A occurs at the same instant as
//C within boolean_while condition.
struct reduce_boolean_while_sub_cond : public
            crule<reduce_boolean_while_sub_cond> {
    reduce_boolean_while_sub_cond() : crule<reduce_boolean_while_sub_cond>::crule("reduce_boolean_while_sub_cond") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//--------------------------------------------------------------------------
//Action reduction rules based on always_succeeds properties
//--------------------------------------------------------------------------

//action_action_if(A B C) -> and_seq(A B) iff get_action_result(A)==true
struct reduce_action_action_if_always_succeeds :
            public crule<reduce_action_action_if_always_succeeds> {
    reduce_action_action_if_always_succeeds() : crule<reduce_action_action_if_always_succeeds>::crule("reduce_action_action_if_always_succeeds") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//action_action_if(A B C) -> exec_seq(A C) iff get_action_result(A)==false
struct reduce_action_action_if_always_fails :
            public crule<reduce_action_action_if_always_fails> {
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//action_while(A) -> A iff get_action_result(A)==false
struct reduce_action_while_always_fails :
            public crule<reduce_action_while_always_fails> {
    reduce_action_while_always_fails() : crule<reduce_action_while_always_fails>::crule("reduce_action_while_always_fails") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//boolean_while(C A) -> action_sucess iff eval(C)==false
//boolean_while(C A) -> A iff eval(C)==true and get_action_result(A)==false
struct reduce_boolean_while_depend_condition :
            public crule<reduce_boolean_while_depend_condition> {
    reduce_boolean_while_depend_condition() : crule<reduce_boolean_while_depend_condition>::crule("reduce_boolean_while_depend_condition") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//and_seq(A1 ... An ... Am) -> and_seq(A1 ... An)
//iff exists n and for all i<n, get_action_result(Ai) = true or undetermined
//and get_action_result(An)==false
struct reduce_sequential_and_always_fails :
            public crule<reduce_sequential_and_always_fails> {
    reduce_sequential_and_always_fails() : crule<reduce_sequential_and_always_fails>::crule("reduce_sequential_and_always_fails") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//or_seq(A1 ... An ... Am) -> or_seq(A1 ... An)
//iff exists n and for all i<n, get_action_result(Ai) = false or undetermined
//and get_action_result(An)==true
struct reduce_sequential_or_always_succeeds :
            public crule<reduce_sequential_or_always_succeeds> {
    reduce_sequential_or_always_succeeds() : crule<reduce_sequential_or_always_succeeds>::crule("reduce_sequential_or_always_succeeds") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//--------------------------------------------------------------------------
//Action reduction rules depending on properties
//--------------------------------------------------------------------------

//reduce 2 consecutive actions to 1 action
struct reduce_idempotent : public crule<reduce_idempotent> {
    reduce_idempotent() : crule<reduce_idempotent>::crule("reduce_idempotent") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//reduce 2 consecutive opposite actions A1 A2
//that is A1 is the reversal of A2
struct reduce_opposite : public crule<reduce_opposite> {
    reduce_opposite() : crule<reduce_opposite>::crule("reduce_opposite") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};


//reduce 2 consecutive actions to 1 action and add the arguments that
//are denoted additive by is_additive(action,int i);
//the argument that are not denoted additive must be equal
struct reduce_additive : public crule<reduce_additive> {
    reduce_additive() : crule<reduce_additive>::crule("reduce_additive") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};


//replace action by action_success if exists 0 in one of its argument
//neutral
struct reduce_zero_neutral : public crule<reduce_zero_neutral> {
    reduce_zero_neutral() : crule<reduce_zero_neutral>::crule("reduce_zero_neutral") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};



//--------------------------------------------------------------------------
//Reduction rules for modular arguments
//--------------------------------------------------------------------------

//action_with_modular_argument(m) -> action_with_modular_argument(reduced(m))
//for example, dummy_modular_action(6.28,-20)-> dummy_modular_action(0,-10)
struct reduce_modular_argument : public crule<reduce_modular_argument> {
    reduce_modular_argument() : crule<reduce_modular_argument>::crule("reduce_modular_argument") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};


//--------------------------------------------------------------------------
// Preconditions checks
//--------------------------------------------------------------------------

//preconditions checks
//for example, if there is a precondition pair (grab drop), then:
// and_seq(bark drop)-> and_seq(bark action_success)
// and_seq(grab bark drop)-> and_seq(grab bark drop)
// and_seq(grab drop drop)-> and_seq(grab drop action_success)
// and_seq(bark drop drop)-> and_seq(bark action_success action_success)
// and_seq(or_seq(grab bark) drop bark)-> and_seq(or_seq(grab bark) drop bark)
// and_seq(or_seq(bark bark) drop bark)-> and_seq(or_seq(bark bark) action_success bark)
struct preconditions_check : public crule<preconditions_check> {
    preconditions_check() : crule<preconditions_check>::crule("preconditions_check") {}
    void operator() (combo_tree& tr, combo_tree::iterator it) const;
};

//auxilary function for the preconditions check
bool reduce_free_post_action (builtin_action pre_a, builtin_action post_a, bool free_pre_action_before, combo_tree& tr, combo_tree::iterator it);

} // ~namespace reduct
} // ~namespace opencog 

#endif
