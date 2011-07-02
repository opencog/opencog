/*
 * opencog/comboreduct/combo/perception.h
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
#ifndef _COMBO_PERCEPTION_H
#define _COMBO_PERCEPTION_H

#include <opencog/util/exceptions.h>

#include "type_tree_def.h"
#include "operator_base.h"

//if the user wants to use the reduct engine it is recommended to
//enable the following macro in order to allow the compiler to detect 
//perception property methods which have not been implemented
#define NO_DEFAULT_PERCEPTION_PROPERTY_METHODS

namespace opencog { namespace combo {

/**
 * perception_base is an abstract class to be implemented
 * to define the set of perceptions, their name (get_name()),
 * their type (get_type_tree(), get_output_type_tree(), etc) and properties
 * (is_symetric(), etc)
 * It is up to the user how to represent the set of actions
 * (std::set, enum, etc)
 * Name, type (and type helpers) are mandatory because of the type
 * checker engine
 * whereas perception properties are only mandatory when using the
 * reduct engine
 * is that case the macro NO_DEFAULT_PERCEPTION_PROPERTY_METHODS should be
 * enabled.
 *
 * For an example of an implementation of that class see ant_perception
 * in files ant_perception.h/cc
 */

class perception_base : public operator_base {
public:
    virtual ~perception_base() {}
  
    //action properties required for reduction
#ifdef NO_DEFAULT_ACTION_PROPERTY_METHODS
    virtual bool is_ultrametric() const = 0;
    virtual bool is_transitive() const = 0;
    virtual bool is_irreflexive() const = 0;
    virtual bool is_reflexive() const = 0;
    virtual bool is_symmetric() const = 0;
    virtual bool is_identity_of_indiscernibles() const = 0;
#else
    virtual bool is_ultrametric() const {
        return false;
    }
    virtual bool is_transitive() const {
        return false;
    }
    virtual bool is_irreflexive() const {
        return false;
    }
    virtual bool is_reflexive() const {
        return false;
    }
    virtual bool is_symmetric() const {
        return false;
    }
    virtual bool is_identity_of_indiscernibles() const {
        return false;
    }
#endif
};

typedef const perception_base* perception;

typedef std::set<perception> perception_set;
typedef perception_set::iterator perception_set_it;
typedef perception_set::const_iterator perception_set_const_it;

} // ~namespace combo

std::ostream& operator<<(std::ostream&, combo::perception);

} // ~namespace opencog

#endif

