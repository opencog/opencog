#ifndef _COMBO_ACTION_H
#define _COMBO_ACTION_H

#include "util/exceptions.h"
#include "comboreduct/combo/type_tree_def.h"

#include <iostream>
#include <vector>
//#include <cassert>
#include "comboreduct/combo/common_def.h"

namespace combo
{

    namespace id {
        enum action {
            sequential_and, sequential_or, sequential_exec,
            action_not, //negate the action_result of an action, that is
            //action_not(action_succeed)=action_failure
            //action_not(action_failure)=action_succeed
            action_if, //boolean in condition,
                       //action branch (for consistency this is action_boolean_if for i/o - moshe)
            action_boolean_if = action_if, //nasty hack but allowed - do not insert anything between this line and the one above
            boolean_action_if, //action_result in condition, boolean branch
            contin_action_if, //action_result in condition, contin branch
            action_action_if, //action_result in condition, action branch
            action_success, action_failure,
            action_while, boolean_while, return_success, repeat_n,
            action_count
        };
    }

    typedef id::action action;

    //return the arity of an action
    //if an argument of the action is arg_list, then the arity is -1
    arity_t get_arity(action aa);

}//~namespace combo


std::ostream& operator<<(std::ostream&, const combo::action&);

#endif
