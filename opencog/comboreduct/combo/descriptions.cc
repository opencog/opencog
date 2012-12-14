/*
 * opencog/comboreduct/combo/descriptions.cc
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

#include "descriptions.h"
#include "../type_checker/type_tree.h"

namespace opencog { namespace combo {

namespace builtin_properties {

using namespace std;

// do not change the ordering of members (because of initialization used below)
struct builtin_description {
    builtin b;
    string builtin_type;
};

// For all builtins the arity must be given here; the ordering of the
// builtins is irrelevant.  This array is used for building an array
// with builtins as indicies, within the singleton class builtin_properties.
// This array should not have any other usages.
//
// ToDo: would be nice to have a more Caml/Haskell style syntax here,
// right?
static const builtin_description bd[] =
{
    // name                     type  
    { id::null_vertex,          "unknown" },
    { id::logical_and,          "->(arg_list(boolean) boolean)" },
    { id::logical_or,           "->(arg_list(boolean) boolean)" },
    { id::logical_not,          "->(boolean boolean)" },
    { id::logical_true,         "boolean" },
    { id::logical_false,        "boolean" },
    { id::plus,                 "->(arg_list(contin) contin)" },
    { id::times,                "->(arg_list(contin) contin)" },
    { id::div,                  "->(contin contin contin)" },
    { id::exp,                  "->(contin contin)" },
    { id::log,                  "->(contin contin)" },
    { id::sin,                  "->(contin contin)" },
    { id::greater_than_zero,    "->(contin boolean)" }, 
    { id::impulse,              "->(boolean contin)" },
    { id::rand,                 "contin" },

    { id::list,                 "->(arg_list(unknown) list)" },
    { id::car,                  "->(list unknown)" },
    { id::cdr,                  "->(list list)" },
    { id::cons,                 "->(unknown list list)" },
    { id::foldr,                "->(unknown unknown list unknown)" },
    { id::foldl,                "->(unknown unknown list unknown)" },
    { id::lambda,               "->(arg_list(unknown) ->)" },
    { id::apply,                "->(-> unknown unknown)" },

    //{ id::ann,                  "->(arg_list(boolean))" },
    //{ id::ann_node,             "->(arg_list(boolean))" },
    //{ id::ann_input,           "boolean" },

    // cond primitive, in the style of lisp/sheme: so
    //     cond(p1 v1 p2 v2 .. pn vn y) 
    // means pattern matching:
    //     if (p1) v1; else if (p2) v2; else if ... else y
    //
    // The 'value' is marked unknown, as it can be of any type.
    // XXX Should probably be "union", yeah?
    { id::cond,               "->(arg_list(boolean unknown) unknown unknown)" },

    // Someday, contin_if will be obsolted/equivalent to above cond ...
    { id::contin_if,            "->(boolean contin contin contin)" },

    // Tests for equality. XXX Someday, we should also allow equality
    // testing for booleans, contin as well, but not today. 
    // Fixing this will require work in typechecker, and use of union
    // type. XXX FIXME...
    { id::equ,                  "->(enum enum boolean)" },
};


    // Constructor  
builtins_properties::builtins_properties()
{
    // setting arities and other properties of single actions
    unsigned int number_of_builtin_descriptions = sizeof(bd)/sizeof(builtin_description);
    OC_ASSERT(number_of_builtin_descriptions == (unsigned int)id::builtin_count,
              "there must be entries for all builtins.");
    // there must be entries for all actions 
    
    type_tree t; 
    
    for (unsigned int i=0; i<number_of_builtin_descriptions; i++) { 
        for (unsigned int j=0; j<i; j++) { 
            OC_ASSERT(bd[i].b!=bd[j].b, 
                      "there must not be two entries for one builtin.");           
            // there must not be two entries for one builtin 
        }
        OC_ASSERT(bd[i].b>=(id::builtin)0, 
                  "must be one of the defined builtins.");
        // must be one of the defined builtins
        OC_ASSERT(bd[i].b<id::builtin_count,
                  "must be one of the defined builtins.");
        // must be one of the defined builtins
        
        std::istringstream is(bd[i].builtin_type);
        is >> t;
        builtin_type_tree[bd[i].b] = t;
        
        type_tree::iterator ty_it = t.begin();  // setting the arity
        arity[bd[i].b] = type_tree_arity(t); 
        
        if (*ty_it==id::lambda_type)
            output_type[bd[i].b] = *ty_it.last_child(); 
        else 
            output_type[bd[i].b] = *ty_it; 
        
        unsigned int index = 0;  
        type_tree_sib_it last_child = t.last_child(ty_it);
        for (type_tree_sib_it sib = ty_it.begin(); sib != last_child; ++sib) {
            argument[bd[i].b][index++] = 
                type_tree(*sib == id::arg_list_type ? sib.begin() : sib);
        }
    }
}

}  // namespace builtin_properties

namespace action_properties {

using namespace std;

// do not change the ordering of members (because of initialization used below)
struct action_description {
    action a;
    string action_type;
};

// For all actions, the arity must be given here; the ordering of the
// actions is irrelevant.  This array is used for building an array
// with actions as indices, within the singleton class action_properties.
// This array should not have any other usages.
static const action_description ad[] =
{
    //action                  type  
    { id::sequential_and,     "->(arg_list(action_result) action_result)" },
    { id::sequential_or,      "->(arg_list(action_result) action_result)" },
    { id::sequential_exec,    "->(arg_list(action_result) action_result)" },
    { id::action_not,         "->(action_result action_result)" },
    { id::action_boolean_if,  "->(boolean action_result action_result action_result)" },
    { id::boolean_action_if,  "->(action_result boolean boolean boolean)" },
    { id::contin_action_if,   "->(action_result contin contin contin)" },
    { id::action_action_if,   "->(action_result action_result action_result action_result)" },
    { id::action_success,     "action_result" },
    { id::action_failure,     "action_result" },
    { id::action_while,       "->(action_result action_result)" },
    { id::boolean_while,      "->(boolean action_result action_result)" },
    { id::return_success,     "action_result" },
    { id::repeat_n,           "->(contin action_result action_result)" }
};


// Constructor  
actions_properties::actions_properties()
{
    // setting arities and other properties of single actions
    unsigned int number_of_action_descriptions = sizeof(ad)/sizeof(action_description);
    OC_ASSERT(number_of_action_descriptions==(unsigned int)id::action_count,
              "there must be entries for all actions.");
    // there must be entries for all actions 
    
    type_tree t; 
    
    for (unsigned int i = 0; i < number_of_action_descriptions; i++) { 
        for (unsigned int j = 0; j < i; j++) { 
            OC_ASSERT(ad[i].a != ad[j].a, 
                      "there must not be two entries for one action.");           
            // there must not be two entries for one action 
        }
        OC_ASSERT(ad[i].a >= (id::action)0, 
                  "must be one of the defined actions.");
        // must be one of the defined actions
        OC_ASSERT(ad[i].a < id::action_count,
                  "must be one of the defined actions.");
        // must be one of the defined actions
        
        std::istringstream is(ad[i].action_type);
        is >> t;
        action_type_tree[ad[i].a] = t;
        
        type_tree::iterator ty_it = t.begin();  // setting the arity
        arity[ad[i].a] = type_tree_arity(t); 

        if (*ty_it == id::lambda_type)
            output_type[ad[i].a] = *ty_it.last_child(); 
        else 
            output_type[ad[i].a] = *ty_it; 

        unsigned int index = 0;
        type_tree_sib_it last_child = t.last_child(ty_it);
        for (type_tree_sib_it sib = ty_it.begin(); sib != last_child; ++sib) {
            argument[ad[i].a][index++] = 
                type_tree(*sib == id::arg_list_type ? sib.begin() : sib);
        }
    }
}

}  // namespace action_properties

}} // ~namespaces combo opencog
