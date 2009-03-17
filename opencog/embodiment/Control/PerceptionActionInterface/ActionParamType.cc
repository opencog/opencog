#include "ActionParamType.h"
#include <LADSUtil/Logger.h>
#include <iostream>

using namespace PerceptionActionInterface;

/*====================================* 
 * Static data members initialization *
 * ===================================*/

// Internal maps 
ActionParamType::Name2ActionParamTypeMap ActionParamType::nameMap;
ActionParamType::Code2ActionParamTypeMap ActionParamType::codeMap;

// Definition of all action type values
const ActionParamType& ActionParamType::BOOLEAN() {
    static ActionParamType* result = new ActionParamType(BOOLEAN_CODE, "boolean");
    return *result;
}
const ActionParamType& ActionParamType::INT() {
    static ActionParamType* result = new ActionParamType(INT_CODE, "int");
    return *result;
}
const ActionParamType& ActionParamType::FLOAT() {
    static ActionParamType* result = new ActionParamType(FLOAT_CODE, "float");
    return *result;
}
const ActionParamType& ActionParamType::STRING() {
    static ActionParamType* result = new ActionParamType(STRING_CODE, "string");
    return *result;
}
const ActionParamType& ActionParamType::VECTOR() {
    static ActionParamType* result = new ActionParamType(VECTOR_CODE, "vector");
    return *result;
}
const ActionParamType& ActionParamType::ROTATION() {
    static ActionParamType* result = new ActionParamType(ROTATION_CODE, "rotation");
    return *result;
}
const ActionParamType& ActionParamType::ENTITY() {
    static ActionParamType* result = new ActionParamType(ENTITY_CODE, "entity");
    return *result;
}

// Constructors 

ActionParamType::ActionParamType() {}; 

ActionParamType::ActionParamType(ActionParamTypeCode _code, const std::string& _name): 
    code(_code), name(_name)  {
    if (ActionParamType::existCode(code)) {
        LADSUtil::cassert(TRACE_INFO, false, "ActionParamType - Duplicate action parameter type code: %d", _code);
    }
    if (ActionParamType::existName(name)) {
    	LADSUtil::cassert(TRACE_INFO, false, "ActionParamType - Duplicate action parameter type name: %s", _name.c_str());
    }
    nameMap[name] = this;
    codeMap[code] = this;
}

// GETTERS 

const std::string& ActionParamType::getName() const { 
    return name; 
}

ActionParamTypeCode ActionParamType::getCode() const { 
    return code; 
}

bool ActionParamType::operator==(const ActionParamType& other) const {
    return (code == other.code);
}

bool ActionParamType::operator!=(const ActionParamType& other) const {
    return (code != other.code);
}


// STATIC METHODS:

void ActionParamType::init() {
    static bool initialized = false;
    if (!initialized) {
        BOOLEAN();
        INT();
        FLOAT();
        STRING();
        VECTOR();
        ROTATION();
        ENTITY();
        
        initialized = true;
    }
} 

const ActionParamType& ActionParamType::getFromName(const std::string& name) throw (LADSUtil::InvalidParamException, std::bad_exception) {
    init();
    Name2ActionParamTypeMap::const_iterator itr = nameMap.find(name);
    if (itr == nameMap.end()) {
        throw LADSUtil::InvalidParamException(TRACE_INFO,
                "ActionParamType - Invalid/unknown ActionParam name: %s\n", name.c_str());
    }
    return *(itr->second);
}

const ActionParamType& ActionParamType::getFromCode(ActionParamTypeCode code) throw (LADSUtil::InvalidParamException, std::bad_exception) {
    init();
    Code2ActionParamTypeMap::const_iterator itr = codeMap.find(code);
    if (itr == codeMap.end()) {
        throw LADSUtil::InvalidParamException(TRACE_INFO,
                "ActionParamType - Invalid/unknown ActionParam code: %d\n", code);
    }
    return *(itr->second);
}

bool ActionParamType::existName(const std::string& name) {
    Name2ActionParamTypeMap::const_iterator itr = nameMap.find(name);
    return (itr != nameMap.end());
}

bool ActionParamType::existCode(ActionParamTypeCode code) {
    Code2ActionParamTypeMap::const_iterator itr = codeMap.find(code);
    return (itr != codeMap.end());
}
std::ostream& operator<<(std::ostream& out, const ActionParamType& arg){
    return (out << arg.getName());
}
