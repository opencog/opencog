/*
 * opencog/embodiment/AvatarComboVocabulary/avatar_operator.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _AVATAR_OPERATOR_H
#define _AVATAR_OPERATOR_H

#include <opencog/comboreduct/combo/operator_base.h>
#include <opencog/comboreduct/combo/type_tree.h>

namespace opencog { namespace combo {

using namespace std;

//this class implement operator_base in a generic way using
//an enum
//enum_count corresponds to the last enum element of OPERATOR_ENUM
//supposely denoting the number of elements
template<typename OPERATOR_ENUM, OPERATOR_ENUM enum_count>
class avatar_operator : public operator_base
{

public:

    //struct for description of name and type
    struct basic_description {
        OPERATOR_ENUM operator_enum;
        string name;
        string type;
    };

protected:

    //enum, i.e. set of operators
    OPERATOR_ENUM _enum;

    //name and type
    std::string _name;
    type_tree _type_tree;
    arity_t _arity;
    type_tree _output_type;
    argument_type_list _arg_type_tree;

    //ctor
    avatar_operator();

    //these 2 methods must be implemented to simply contain the address of
    //the start of an array containing all basic descriptions
    //and the number of entries
    virtual const basic_description* get_basic_description_array() const = 0;
    virtual unsigned int get_basic_description_array_count() const = 0;

    //this is called to fill name and type using the basic_description
    //array returned by get_basic_description_array
    void set_basic_description(OPERATOR_ENUM oe);

public:
    OPERATOR_ENUM get_enum() const;

};

template<typename OPERATOR_ENUM, OPERATOR_ENUM enum_count>
avatar_operator<OPERATOR_ENUM, enum_count>::avatar_operator()
{
    _enum = enum_count;
    _name = "UNDEFINED_OPERATOR";
    _arity = 0;
    _output_type = type_tree(id::ill_formed_type);
}

template<typename OPERATOR_ENUM, OPERATOR_ENUM enum_count>
void avatar_operator<OPERATOR_ENUM, enum_count>::set_basic_description(OPERATOR_ENUM oe)
{
    const basic_description* bd = get_basic_description_array();
    unsigned int bd_count = get_basic_description_array_count();
    OC_ASSERT(bd_count == (unsigned int)enum_count,
                     "there must be entries for all perceptions.");
    bool found = false;
    for (unsigned int i = 0; i < bd_count && !found; ++i) {
        if (bd[i].operator_enum == oe) {
            found = true;
            //setting perception name
            _name = bd[i].name;
            //setting perception type tree
            std::istringstream is(bd[i].type);
            try {
                is >> _type_tree;
            } catch (opencog::InconsistenceException& ie) {
                std::cout << "WARNING : there must be a problem with the type description of " << _name << ", as the interpretation of the type string : " << "\"" << is.str() << "\"" << " has raised the following exception : " << ie.getMessage() << std::endl;
            }
            //setting arity
            _arity = type_tree_arity(_type_tree);
            //setting output type
            _output_type = type_tree_output_type_tree(_type_tree);
            //setting input argument type trees
            _arg_type_tree = type_tree_input_arg_types(_type_tree);
        }
    }
    OC_ASSERT(found,
                     "avatar_perception with enum %d has not been found in pbd",
                     oe);
}

template<typename OPERATOR_ENUM, OPERATOR_ENUM enum_count>
OPERATOR_ENUM avatar_operator<OPERATOR_ENUM, enum_count>::get_enum() const
{
    return _enum;
}

}} // ~namespaces combo opencog

#endif
