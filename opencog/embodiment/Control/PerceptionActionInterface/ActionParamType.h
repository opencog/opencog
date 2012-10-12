/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionParamType.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef _ACTION_PARAM_TYPE_H_
#define _ACTION_PARAM_TYPE_H_
/**
 * ActionParamType.h:
 *
 * Defines the types of action paramters, according with XSD, as follows:
 * <xsd:enumeration value="boolean"/>
 * <xsd:enumeration value="int"/>
 * <xsd:enumeration value="float"/>
 * <xsd:enumeration value="string"/>
 * <xsd:enumeration value="vector"/>
 * <xsd:enumeration value="rotation"/>
 * <xsd:enumeration value="list"/> => not used in PB side
 * <xsd:enumeration value="entity"/>
 * <xsd:enumeration value="nil"/> => not used in PB side
 */

#include <string>
#include <map>
#include <sstream>
#include <iostream>
#include <exception>

#include <opencog/util/exceptions.h>

namespace opencog { namespace pai {

/**
 * Numeric codes for each action parameter type:
 */
enum ActionParamTypeCode {
    BOOLEAN_CODE,
    INT_CODE,
    FLOAT_CODE,
    STRING_CODE,
    VECTOR_CODE,
    ROTATION_CODE,
    ENTITY_CODE,
    FUZZY_INTERVAL_INT_CODE,
    FUZZY_INTERVAL_FLOAT_CODE,

    NUMBER_OF_ACTION_PARAM_TYPES // This must be at the end of the enumeration in order to count the number of action types.
};


/**
 * ActionParamType class. Wrappers both primitive's code and name in an object
 */
class ActionParamType
{

public:
    // Object methods
    const std::string& getName() const;
    ActionParamTypeCode getCode() const;

    bool operator==(const ActionParamType& other) const;
    bool operator!=(const ActionParamType& other) const;

    // Class methods
    static void init();

    /**
     * @throws InvalidParamException if an invalid Action parameter type string is given
     */
    static const ActionParamType& getFromName(const std::string& name) throw (opencog::InvalidParamException, std::bad_exception);
    /**
     * @throws InvalidParamException if an invalid Action parameter type code is given
     */
    static const ActionParamType& getFromCode(ActionParamTypeCode code) throw (opencog::InvalidParamException, std::bad_exception);

    static const ActionParamType& BOOLEAN();
    static const ActionParamType& INT();
    static const ActionParamType& FLOAT();
    static const ActionParamType& STRING();
    static const ActionParamType& VECTOR();
    static const ActionParamType& ROTATION();
    static const ActionParamType& ENTITY();
    static const ActionParamType& FUZZY_INTERVAL_INT();
    static const ActionParamType& FUZZY_INTERVAL_FLOAT();

private:
    // Attributes
    ActionParamTypeCode code;
    std::string name;

    /**
     * Empty constructor
     */
    ActionParamType();

    /**
     * Constructor
     */
    ActionParamType(ActionParamTypeCode _code, const std::string& _name);

    // Static data structure
    typedef std::map<std::string, ActionParamType*> Name2ActionParamTypeMap;
    typedef std::map<ActionParamTypeCode, ActionParamType*> Code2ActionParamTypeMap;
    static Name2ActionParamTypeMap& nameMap();
    static Code2ActionParamTypeMap& codeMap();

    // private static methods
    static bool existName(const std::string& name);
    static bool existCode(ActionParamTypeCode code);
};

std::ostream& operator<<(std::ostream& out, const ActionParamType& arg);


struct ActionParamStruct
{
    std::string paramName;
    ActionParamTypeCode paramType;
    std::string paramValue;
};

} } // namespace opencog::pai

#endif //_ACTION_PARAM_TYPE_H_
