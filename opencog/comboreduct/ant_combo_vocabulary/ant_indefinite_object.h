/*
 * opencog/comboreduct/ant_combo_vocabulary/ant_indefinite_object.h
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
#ifndef _ANT_INDEFINITE_OBJECT_H
#define _ANT_INDEFINITE_OBJECT_H

#include <opencog/util/numeric.h>

#include <opencog/comboreduct/combo/indefinite_object.h>
#include "ant_operator.h"

namespace opencog { namespace combo {

namespace id {
    enum ant_indefinite_object_enum {
        ant_indefinite_object_count
    };
}

typedef id::ant_indefinite_object_enum ant_indefinite_object_enum;

/*********************************************************************
 *   Arrays containing indefinite_object name type and properties    *
 *                 to be edited by the developer                     *
 *********************************************************************/

namespace ant_indefinite_object_properties {

    //struct for description of name and type
    typedef ant_operator<ant_indefinite_object_enum, id::ant_indefinite_object_count>::basic_description indefinite_object_basic_description;

    //empty but kept for example
    static const indefinite_object_basic_description iobd[] = {
        //indefinite_object        name                   type
    };

}//~namespace ant_perception_properties

//ant_indefinite_object both derive
//from indefinite_object_base and ant_operator
class ant_indefinite_object : public ant_operator<ant_indefinite_object_enum, id::ant_indefinite_object_count>, public indefinite_object_base {

private:

    //private methods

    //ctor
    ant_indefinite_object();

    const basic_description * get_basic_description_array() const;
    unsigned int get_basic_description_array_count() const;

    static const ant_indefinite_object* init_indefinite_object();
    void set_indefinite_object(ant_indefinite_object_enum);

public:
    //name
    const std::string& get_name() const;

    //type_tree
    const type_tree& get_type_tree() const;

    //helper methods for fast access type properties
    //number of arguments that takes the operator
    arity_t arity() const;
    //return the type node of the operator
    type_tree get_output_type_tree() const;
    //return the type tree of the argument of index i
    //if the operator has arg_list(T) as last input argument
    //then it returns always T past that index
    const type_tree& get_input_type_tree(arity_t i) const;

    //return a pointer of the static action_symbol corresponding
    //to a given name string
    //if no such action_symbol exists then return NULL pointer
    static indefinite_object get_instance(const std::string& name);

    //return a pointer of the static ant_perception_action corresponding
    //to a given ant_perception_enum
    static indefinite_object get_instance(ant_indefinite_object_enum);

};

}} // ~namespaces combo opencog

#endif
