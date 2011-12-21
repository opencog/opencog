/*
 * opencog/comboreduct/combo/type_tree_def.h
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
#ifndef _COMBO_TYPE_TREE_DEF_H
#define _COMBO_TYPE_TREE_DEF_H

#include <opencog/util/tree.h>

namespace opencog { namespace combo {

namespace id {
  enum type_node {
    //type operators
    lambda_type, //example : lambda_type(T1 T2 T3)
                 //represents a function that takes
                 //T1 and T2 argument's type and returns
                 //an output of type T3
    application_type, //represent the application of a function with
                      //its arguments
    union_type,
    arg_list_type,         //denote a 0 or more argument(s) of a given type
                           //example : arg_list(T) corresponds to a list of
                           //0 or more elements of type T
    //elementary types
    boolean_type,
    contin_type,
    action_result_type,
    definite_object_type,
    action_definite_object_type, //like definite_object but contains the suffix _action
    indefinite_object_type,
    message_type,
    action_symbol_type,
    wild_card_type,
    ann_type, //for evolving anns
    //unknown type
    unknown_type,  //it is the uber type all types inherite from unkown
                   //but ill_formed_type
    //ill formed
    ill_formed_type, //when the type is just wrong

    //argument, below is a small hack to avoid using union or variant
    //the rest of the integers enumerate variables
    //i.e. argument_type corresponds to #1, variable_type+1 corresponds to #2
    //and so on
    argument_type
  };
}
typedef id::type_node type_node;

//structure that codes the type of a tree
typedef opencog::tree<type_node> type_tree;

typedef type_tree::iterator type_tree_pre_it;
typedef type_tree::sibling_iterator type_tree_sib_it;

//list of argument types
typedef std::vector<type_tree> type_tree_seq;
typedef type_tree_seq::iterator type_tree_seq_it;
typedef type_tree_seq::const_iterator type_tree_seq_cit;
const static type_tree_seq empty_tts;

//check whether a given type_node represents an argument type
bool is_argument_type(type_node n);
//return the idx (as defined in class argument in vertex.h) corresponding to
//a given type_node
//it is assumed that n is a argument type
unsigned int arg_to_idx(type_node n);

}} // ~namespace combo opencog

#endif
