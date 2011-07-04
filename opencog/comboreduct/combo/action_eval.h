/*
 * opencog/comboreduct/combo/action_eval.h
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
#ifndef _COMBO_ACTION_EVAL_H
#define _COMBO_ACTION_EVAL_H

#include "util/exceptions.h"
#include "util/tree.h"
#include "util/RandGen.h"

#include "comboreduct/combo/vertex.h"
#include "comboreduct/crutil/exception.h"
#include "comboreduct/combo/using.h"
#include "comboreduct/combo/type_tree.h"

#include <exception>

namespace opencog { namespace combo {
//inline boost::variant<vertex,combo_tree::iterator>& binding(int idx) {
//  static hash_map<int,boost::variant<vertex,combo_tree::iterator> > map; //to support lazy evaluation, can also bind to a subtree
//  return map[idx];
//}

template<typename It>
vertex action_eval_throws(RandGen& rng, It it) throw(EvalException, AssertionException, std::bad_exception) {
    typedef typename It::sibling_iterator sib_it;
    //argument
    //WARNING : we assume the argument is necessarily an action and therefor
    //never has a negative index
    if(is_argument(*it)) {
        arity_t idx=get_argument(*it).idx;
        //assumption : when idx is negative the argument is necessary boolean
        OC_ASSERT(idx > 0, "argument is necessarily an action and therefore never has a neg idx.");
        if (const vertex* v=boost::get<const vertex>(&binding(idx)))
            return *v;
        else
            return action_eval_throws(rng, boost::get<combo_tree::iterator>(binding(idx)));
    }
    //action sequence
    if(*it==id::sequential_and) {
        for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
            if(action_eval_throws(rng, sib) != id::action_success)
                return id::action_failure;
        }
        return id::action_success;
    }
    else if(*it==id::sequential_or) {
        for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
            if(action_eval_throws(rng, sib) == id::action_success)
                return id::action_success;
        }
        return id::action_failure;
    }
    else if(*it==id::sequential_exec) {
        for(sib_it sib = it.begin(); sib != it.end(); ++sib)
            action_eval_throws(rng, sib);
        return id::action_success;
    }
    //action_if
    else if(*it==id::boolean_action_if) {
        OC_ASSERT(it.number_of_children()==3, 
                  "combo_tree node should have exactly three children (id::boolean_if).");
        sib_it sib = it.begin();
        vertex vcond = action_eval_throws(rng, sib);
        ++sib;
        if(vcond==id::action_success) {
            //TODO : check that b1 and b2 have type boolean
            return eval_throws(rng, sib);
        }
        else {
            ++sib;
            return eval_throws(rng, sib);
        }
    }
    else if(*it==id::contin_action_if) {
        OC_ASSERT(it.number_of_children()==3,
                  "combo_tree node should have exactly three children (id::contin_action_if).");
        sib_it sib = it.begin();
        vertex vcond = action_eval_throws(rng, sib);
        ++sib;
        if(vcond==id::action_success) {
            //TODO : check that b1 and b2 have type contin
            return eval_throws(rng, sib);
        }
        else {
            ++sib;
            return eval_throws(rng, sib);
        }
    }
    else if(*it==id::action_action_if) {
        OC_ASSERT(it.number_of_children()==3,
                  "combo_tree node should have exactly three children (id::contin_action_if).");
        sib_it sib = it.begin();
        vertex vcond = action_eval_throws(rng, sib);
        ++sib;
        if(vcond==id::action_success) {
            //TODO : check that b1 and b2 have type action
            return action_eval_throws(rng, sib);
        }
        else {
            ++sib;
            return action_eval_throws(rng, sib);
        }
    }
    else {
        return *it;
    }
}

} // ~namespace combo
} // ~namespace opencog 

#endif
