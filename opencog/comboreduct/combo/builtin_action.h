/*
 * opencog/comboreduct/combo/builtin_action.h
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
#ifndef _COMBO_BUILTIN_ACTION_H
#define _COMBO_BUILTIN_ACTION_H

#include <opencog/util/exceptions.h>

#include "type_tree_def.h"
#include "common_def.h"
#include <iostream>
#include <vector>
//#include <cassert>
#include "operator_base.h"

//if the user wants to use the reduct engine it is recommended to
//enable the following macro in order to allow the compiler to detect 
//action property methods which have not been implemented
#define NO_DEFAULT_ACTION_PROPERTY_METHODS

namespace opencog { namespace combo {

  /**
 * builtin_action_base is an abstract class to be implemented
 * to define the set of builtin actions, their name (get_name()),
 * their type (get_type_tree(), get_output_type_tree, etc)) and properties
 * (is_reversible(), etc)
 * It is up to the user how to represent the set of actions
 * (std::set, enum, etc)
 * Name, type and arity are mandatory because of the type checker engine
 * whereas properties are only mandatory when using the reduct engine
 * is that case the macro NO_DEFAULT_ACTION_PROPERTY_METHODS should be
 * enabled.
 *
 * For an example of an implementation of that class see ant_builtin_action
 * in files ant_builtin_action.h/cc
 */

class builtin_action_base : public operator_base {
public:
    virtual ~builtin_action_base() {}

    //action properties required for reduction
#ifdef NO_DEFAULT_ACTION_PROPERTY_METHODS
    virtual bool is_reversible() const = 0;
    virtual bool always_succeeds() const = 0;
    virtual const builtin_action_base* get_reversal() const = 0;
    virtual bool is_idempotent() const = 0;
    virtual bool is_additive(arity_t index) const = 0;
    virtual bool exists_additive_argument() const = 0;
    virtual bool is_zero_neutral(arity_t index) const = 0;
    virtual bool exists_zero_neutral_argument() const = 0;
    virtual bool is_modulo(arity_t index) const = 0;
    virtual double modulo_min(arity_t index) const = 0;
    virtual double modulo_max(arity_t index) const = 0;
    virtual const std::set<const builtin_action_base*> preconditions() const = 0;
#else
    virtual bool is_reversible() const {
        return false;
    }
    virtual bool always_succeeds() const {
        return false;
    }
    virtual const builtin_action_base* get_reversal() const {
        return this;
    }
    virtual bool is_idempotent() const {
        return false;
    }
    virtual bool is_additive(arity_t index) const {
        return false;
    }
    virtual bool is_additive() const {
        return false;
    }
    virtual bool is_zero_neutral(arity_t index) const {
        return false;
    }
    virtual bool is_zero_neutral() const {
        return false;
    }
    virtual bool is_modulo(arity_t index) const {
        return false;
    }
    virtual double modulo_min(arity_t index) const {
        return 0.0;
    }
    virtual double modulo_max(arity_t index) const {
        return 0.0;
    }
    virtual const std::set<const builtin_action_base*> preconditions() const {
        static const std::set<const builtin_action_base*> tmp;
        return tmp;
    }
#endif
};

typedef const builtin_action_base* builtin_action;
typedef std::set<builtin_action> builtin_action_set;
typedef builtin_action_set::iterator builtin_action_set_it;
typedef builtin_action_set::const_iterator builtin_action_set_const_it;

} // ~namespace combo

std::ostream& operator<<(std::ostream&, combo::builtin_action);

} // ~namespace opencog

#endif
