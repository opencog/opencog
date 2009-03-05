#ifndef _COMBO_TYPE_TREE_DEF_H
#define _COMBO_TYPE_TREE_DEF_H

#include <LADSUtil/tree.h>

namespace combo {

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
  typedef LADSUtil::tree<type_node> type_tree;

  typedef type_tree::iterator type_tree_pre_it;
  typedef type_tree::sibling_iterator type_tree_sib_it;

  //list of argument types
  typedef std::vector<type_tree> argument_type_list;
  typedef argument_type_list::iterator argument_type_list_it;
  typedef argument_type_list::const_iterator argument_type_list_const_it;
  const static argument_type_list empty_atl;

  //check whether a given type_node represents an argument type
  bool is_argument_type(type_node n);
  //return the idx (as defined in class argument in vertex.h) corresponding to
  //a given type_node
  //it is assumed that n is a argument type
  unsigned int arg_to_idx(type_node n);
}

#endif
