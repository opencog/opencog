/*
 * opencog/comboreduct/combo/descriptions.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Predrag Janicic
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
#ifndef _COMBO_DESCRIPTIONS_H
#define _COMBO_DESCRIPTIONS_H

#include <opencog/util/numeric.h>

#include "action.h"
#include "../type_checker/type_tree.h"

#include <opencog/util/tree.h>

namespace opencog { namespace combo {

namespace builtin_properties {

const char maximal_builtin_arity=3;

// This singleton class stores properties of builtins.
// Within the constructor, properties, given above in an easily
// editable form, are read and stored in a suitable form, with fast
// access and small ammount of used memory
class builtins_properties
{
public:
    static builtins_properties& instance() {
        static builtins_properties singleton;
        return singleton;
    }

    // returns the arity of b 
    char builtin_arity(builtin b) { return arity[b]; }
    
    // returns properties of i-th argument of b
    type_tree builtin_argument(builtin b, unsigned char i) { return argument[b][i]; }    
    
    // returns the arity of b 
    type_tree type_tree_of_builtin(builtin b) { return builtin_type_tree[b]; }
    
    // returns the arity of b 
    id::type_node output_type_of_builtin(builtin b) { return output_type[b]; }
    
    
private:
    char arity[id::builtin_count];
    id::type_node output_type[id::builtin_count];
    type_tree builtin_type_tree[id::builtin_count];
    type_tree argument[id::builtin_count][maximal_builtin_arity];

    // Constructor  
    builtins_properties();
    builtins_properties(const builtins_properties&);
    builtins_properties& operator=(const builtins_properties&);
}; 

}  // namespace builtin_properties

namespace action_properties {

const char maximal_action_arity=3;

// This singleton class stores properties of actions.
// Within the constructor, properties, given above in an easily
// editable form, are read and stored in a suitable form, with
// fast access and small ammount of used memory
class actions_properties
{
public:
    static actions_properties& instance() {
        static actions_properties singleton;
        return singleton;
    }

    // returns the arity of a 
    char action_arity(action a) { return arity[a]; }
    
    // returns properties of i-th argument of a
    type_tree action_argument(action a, unsigned char i) { return argument[a][i]; }    
    
    // returns the arity of a 
    type_tree type_tree_of_action(action a) { return action_type_tree[a]; }
    
    // returns the arity of a 
    id::type_node output_type_of_action(action a) { return output_type[a]; }

private:
    char arity[id::action_count];
    id::type_node output_type[id::action_count];
    type_tree action_type_tree[id::action_count];
    type_tree argument[id::action_count][maximal_action_arity];
    

    // Constructor  
    actions_properties();
    actions_properties(const actions_properties&);
    actions_properties& operator=(const actions_properties&);
}; 

}  // namespace action_properties

}} // ~namespaces combo opencog

#endif // _COMBO_DESCRIPTIONS_H
