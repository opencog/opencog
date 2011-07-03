/*
 * opencog/comboreduct/combo/action.h
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
#ifndef _COMBO_ACTION_H
#define _COMBO_ACTION_H

#include <opencog/util/exceptions.h>
#include "type_tree_def.h"

#include <iostream>
#include <vector>
//#include <cassert>
#include "common_def.h"

namespace opencog { namespace combo {

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

std::ostream& operator<<(std::ostream&, const action&);

} // ~namespace combo
} // ~namespace opencog

#endif
