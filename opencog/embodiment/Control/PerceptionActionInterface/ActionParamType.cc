/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionParamType.cc
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

#include "ActionParamType.h"
#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>
#include <iostream>

using namespace opencog::pai;

/*====================================*
 * Static data members initialization *
 * ===================================*/

// Internal maps
ActionParamType::Name2ActionParamTypeMap& ActionParamType::nameMap() {
    static Name2ActionParamTypeMap nameMap;
    return nameMap;
}

ActionParamType::Code2ActionParamTypeMap& ActionParamType::codeMap() {
    static Code2ActionParamTypeMap codeMap;
    return codeMap;
}

// Definition of all action type values
const ActionParamType& ActionParamType::BOOLEAN()
{
    static ActionParamType* result = new ActionParamType(BOOLEAN_CODE, "boolean");
    return *result;
}
const ActionParamType& ActionParamType::INT()
{
    static ActionParamType* result = new ActionParamType(INT_CODE, "int");
    return *result;
}
const ActionParamType& ActionParamType::FLOAT()
{
    static ActionParamType* result = new ActionParamType(FLOAT_CODE, "float");
    return *result;
}
const ActionParamType& ActionParamType::STRING()
{
    static ActionParamType* result = new ActionParamType(STRING_CODE, "string");
    return *result;
}
const ActionParamType& ActionParamType::VECTOR()
{
    static ActionParamType* result = new ActionParamType(VECTOR_CODE, "vector");
    return *result;
}
const ActionParamType& ActionParamType::ROTATION()
{
    static ActionParamType* result = new ActionParamType(ROTATION_CODE, "rotation");
    return *result;
}
const ActionParamType& ActionParamType::ENTITY()
{
    static ActionParamType* result = new ActionParamType(ENTITY_CODE, "entity");
    return *result;
}

const ActionParamType& ActionParamType::FUZZY_INTERVAL_INT()
{
    static ActionParamType* result = new ActionParamType(FUZZY_INTERVAL_INT_CODE, "fuzzy_interval_int");
    return *result;
}

const ActionParamType& ActionParamType::FUZZY_INTERVAL_FLOAT()
{
    static ActionParamType* result = new ActionParamType(FUZZY_INTERVAL_FLOAT_CODE, "fuzzy_interval_float");
    return *result;
}

// Only the INT,FLOAT,FUZZY_INTERVAL_INT and FUZZY_INTERVAL_FLOAT are Numberic type
bool ActionParamType::isNumbericValueType(ActionParamTypeCode valueTypeCode)
{
    if ( (valueTypeCode == INT_CODE ) ||
         (valueTypeCode == FLOAT_CODE ) ||
         (valueTypeCode == FUZZY_INTERVAL_INT_CODE ) ||
         (valueTypeCode == FUZZY_INTERVAL_FLOAT_CODE ) )
        return true;
    else
        return false;
}


// Constructors

ActionParamType::ActionParamType() {};

ActionParamType::ActionParamType(ActionParamTypeCode _code, const std::string& _name):
        code(_code), name(_name)
{
    if (ActionParamType::existCode(code)) {
        OC_ASSERT(false, "ActionParamType - Duplicate action parameter type code: %d", _code);
    }
    if (ActionParamType::existName(name)) {
        OC_ASSERT(false, "ActionParamType - Duplicate action parameter type name: %s", _name.c_str());
    }
    nameMap()[name] = this;
    codeMap()[code] = this;
}

// GETTERS

const std::string& ActionParamType::getName() const
{
    return name;
}

ActionParamTypeCode ActionParamType::getCode() const
{
    return code;
}

bool ActionParamType::operator==(const ActionParamType& other) const
{
    return (code == other.code);
}

bool ActionParamType::operator!=(const ActionParamType& other) const
{
    return (code != other.code);
}


// STATIC METHODS:

void ActionParamType::init()
{
    static bool initialized = false;
    if (!initialized) {
        BOOLEAN();
        INT();
        FLOAT();
        STRING();
        VECTOR();
        ROTATION();
        ENTITY();
        FUZZY_INTERVAL_INT();
        FUZZY_INTERVAL_FLOAT();

        initialized = true;
    }
}

const ActionParamType& ActionParamType::getFromName(const std::string& name) throw (opencog::InvalidParamException, std::bad_exception)
{
    init();
    Name2ActionParamTypeMap::const_iterator itr = nameMap().find(name);
    if (itr == nameMap().end()) {
        throw opencog::InvalidParamException(TRACE_INFO,
                                             "ActionParamType - Invalid/unknown ActionParam name: %s\n", name.c_str());
    }
    return *(itr->second);
}

const ActionParamType& ActionParamType::getFromCode(ActionParamTypeCode code) throw (opencog::InvalidParamException, std::bad_exception)
{
    init();
    Code2ActionParamTypeMap::const_iterator itr = codeMap().find(code);
    if (itr == codeMap().end()) {
        throw opencog::InvalidParamException(TRACE_INFO,
                                             "ActionParamType - Invalid/unknown ActionParam code: %d\n", code);
    }
    return *(itr->second);
}

bool ActionParamType::existName(const std::string& name)
{
    Name2ActionParamTypeMap::const_iterator itr = nameMap().find(name);
    return (itr != nameMap().end());
}

bool ActionParamType::existCode(ActionParamTypeCode code)
{
    Code2ActionParamTypeMap::const_iterator itr = codeMap().find(code);
    return (itr != codeMap().end());
}
std::ostream& operator<<(std::ostream& out, const ActionParamType& arg)
{
    return (out << arg.getName());
}
