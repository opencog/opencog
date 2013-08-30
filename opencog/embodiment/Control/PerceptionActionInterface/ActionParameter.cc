/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionParameter.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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


#include "ActionParameter.h"
#include "PVPXmlConstants.h"
#include "PAIUtils.h"

#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>


using namespace opencog::pai;

ParamValue opencog::pai::UNDEFINED_VALUE = "UNDEFINED_VALUE";

Entity Entity::NON_Entity = Entity("none","none");

ActionParameter::ActionParameter() : type(ActionParamType::STRING())
{
    PAIUtils::initializeXMLPlatform();
}

ActionParameter::ActionParameter(const string& _name, const ActionParamType& _type, const ParamValue& _value) :
        name(_name), type(_type), value(_value)
{
    PAIUtils::initializeXMLPlatform();
    // validates the value according to the type:
    switch (type.getCode()) {
    case BOOLEAN_CODE:
    case INT_CODE:
    case FLOAT_CODE:
    case STRING_CODE:
        OC_ASSERT(isStringValue(), "ActionParameter constructor: got invalid value for parameter of the type: %s\n", _type.getName().c_str());
        break;
    case VECTOR_CODE:
        OC_ASSERT(isVectorValue(), "ActionParameter constructor: got invalid value for parameter of the type: %s\n", _type.getName().c_str());
        break;
    case ROTATION_CODE:
        OC_ASSERT(isRotationValue(), "ActionParameter constructor: got invalid value for parameter of the type: %s\n", _type.getName().c_str());
        break;
    case ENTITY_CODE:
        OC_ASSERT(isEntityValue(), "ActionParameter constructor: got invalid value for parameter of the type: %s\n", _type.getName().c_str());
        break;
    case FUZZY_INTERVAL_INT_CODE:
        OC_ASSERT(isFuzzyIntervalIntValue(), "ActionParameter constructor: got invalid value for parameter of the type: %s\n", _type.getName().c_str());
        break;
    case FUZZY_INTERVAL_FLOAT_CODE:
        OC_ASSERT(isFuzzyIntervalFloatValue(), "ActionParameter constructor: got invalid value for parameter of the type: %s\n", _type.getName().c_str());
        break;
    default:
        OC_ASSERT(false, "ActionParameter constructor: got invalid parameter type: %s\n", _type.getName().c_str());
        break;
    }

    valueString = stringRepresentation();
}

ActionParameter::~ActionParameter()
{
}



void ActionParameter::assignValue(const ParamValue& newValue)
{
    value = newValue;
    valueString = stringRepresentation();
}

string ActionParameter::getName() const
{
    return name;
}

const ActionParamType& ActionParameter::getType() const
{
    return type;
}

ParamValue& ActionParameter::getValue()
{
    return value;
}

XERCES_CPP_NAMESPACE::DOMElement* ActionParameter::createPVPXmlElement(XERCES_CPP_NAMESPACE::DOMDocument* doc,
        XERCES_CPP_NAMESPACE::DOMElement* parent) const
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];
    XERCES_CPP_NAMESPACE::XMLString::transcode(PARAMETER_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    XERCES_CPP_NAMESPACE::DOMElement *param = doc->createElement(tag);

    XERCES_CPP_NAMESPACE::XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLCh* nameStr = XERCES_CPP_NAMESPACE::XMLString::transcode(name.c_str());
    param->setAttribute(tag, nameStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&nameStr);

    XERCES_CPP_NAMESPACE::XMLString::transcode(TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLCh* typeStr = XERCES_CPP_NAMESPACE::XMLString::transcode(type.getName().c_str());
    param->setAttribute(tag, typeStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&typeStr);

    /// value attribute depends on its type
    if (isStringValue()) {
        XERCES_CPP_NAMESPACE::XMLString::transcode(VALUE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* valueStr = XERCES_CPP_NAMESPACE::XMLString::transcode(getStringValue().c_str());
        param->setAttribute(tag, valueStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&valueStr);
    } else if (isRotationValue()) {
        const Rotation& r = getRotationValue();
        XERCES_CPP_NAMESPACE::XMLString::transcode(ROTATION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        XERCES_CPP_NAMESPACE::DOMElement *rotate = doc->createElement(tag);

        XERCES_CPP_NAMESPACE::XMLString::transcode(PITCH_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* pitchStr = XERCES_CPP_NAMESPACE::XMLString::transcode(opencog::toString(r.pitch).c_str());
        rotate->setAttribute(tag, pitchStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&pitchStr);

        XERCES_CPP_NAMESPACE::XMLString::transcode(ROLL_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* rollStr = XERCES_CPP_NAMESPACE::XMLString::transcode(opencog::toString(r.roll).c_str());
        rotate->setAttribute(tag, rollStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&rollStr);

        XERCES_CPP_NAMESPACE::XMLString::transcode(YAW_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* yawStr = XERCES_CPP_NAMESPACE::XMLString::transcode(opencog::toString(r.yaw).c_str());
        rotate->setAttribute(tag, yawStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&yawStr);

        param->appendChild(rotate);
    } else if (isVectorValue()) {
        const Vector& c = getVectorValue();
        XERCES_CPP_NAMESPACE::XMLString::transcode(VECTOR_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        XERCES_CPP_NAMESPACE::DOMElement *position = doc->createElement(tag);

        XERCES_CPP_NAMESPACE::XMLString::transcode(X_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* xStr = XERCES_CPP_NAMESPACE::XMLString::transcode(opencog::toString(c.x).c_str());
        position->setAttribute(tag, xStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&xStr);

        XERCES_CPP_NAMESPACE::XMLString::transcode(Y_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* yStr = XERCES_CPP_NAMESPACE::XMLString::transcode(opencog::toString(c.y).c_str());
        position->setAttribute(tag, yStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&yStr);

        XERCES_CPP_NAMESPACE::XMLString::transcode(Z_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* zStr = XERCES_CPP_NAMESPACE::XMLString::transcode(opencog::toString(c.z).c_str());
        position->setAttribute(tag, zStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&zStr);

        param->appendChild(position);
    } else if (isEntityValue()) {
        const Entity& e = getEntityValue();
        XERCES_CPP_NAMESPACE::XMLString::transcode(ENTITY_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        XERCES_CPP_NAMESPACE::DOMElement *entity = doc->createElement(tag);

        XERCES_CPP_NAMESPACE::XMLString::transcode(ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* idStr = XERCES_CPP_NAMESPACE::XMLString::transcode(PAIUtils::getExternalId(e.id.c_str()).c_str());
        entity->setAttribute(tag, idStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&idStr);

        XERCES_CPP_NAMESPACE::XMLString::transcode(TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* typeStr = XERCES_CPP_NAMESPACE::XMLString::transcode(e.type.c_str());
        entity->setAttribute(tag, typeStr);
        XERCES_CPP_NAMESPACE::XMLString::release(&typeStr);

        param->appendChild(entity);
    }
    parent->appendChild(param);

    return param;
}

bool ActionParameter::isStringValue() const
{
    return boost::get<string>(&value);
}

bool ActionParameter::isRotationValue() const
{
    return boost::get<Rotation>(&value);
}

bool ActionParameter::isVectorValue() const
{
    return boost::get<Vector>(&value);
}

bool ActionParameter::isEntityValue() const
{
    return boost::get<Entity>(&value);
}

bool ActionParameter::isFuzzyIntervalIntValue() const
{
    return boost::get<FuzzyIntervalInt>(&value);
}

bool ActionParameter::isFuzzyIntervalFloatValue() const
{
    return boost::get<FuzzyIntervalFloat>(&value);
}

bool ActionParameter::isStringValue(ParamValue value)
{
    return boost::get<string>(&value);
}

bool ActionParameter::isRotationValue(ParamValue value)
{
    return boost::get<Rotation>(&value);
}

bool ActionParameter::isVectorValue(ParamValue value)
{
    return boost::get<Vector>(&value);
}

bool ActionParameter::isEntityValue(ParamValue value)
{
    return boost::get<Entity>(&value);
}

bool ActionParameter::isFuzzyIntervalIntValue(ParamValue value)
{
    return boost::get<FuzzyIntervalInt>(&value);
}

bool ActionParameter::isFuzzyIntervalFloatValue(ParamValue value)
{
    return boost::get<FuzzyIntervalFloat>(&value);
}


const string& ActionParameter::getStringValue() const
{
    return get<string>(value);
}

const Rotation& ActionParameter::getRotationValue() const
{
    return get<Rotation>(value);
}

const Vector& ActionParameter::getVectorValue() const
{
    return get<Vector>(value);
}

const Entity& ActionParameter::getEntityValue() const
{
    return get<Entity>(value);
}

const FuzzyIntervalInt& ActionParameter::getFuzzyIntervalIntValue() const
{
    return get<FuzzyIntervalInt>(value);
}

const FuzzyIntervalFloat& ActionParameter::getFuzzyIntervalFloatValue() const
{
    return get<FuzzyIntervalFloat>(value);
}


const string& ActionParameter::getStringValue(ParamValue value)
{
    return get<string>(value);
}

const Rotation& ActionParameter::getRotationValue(ParamValue value)
{
    return get<Rotation>(value);
}

const Vector& ActionParameter::getVectorValue(ParamValue value)
{
    return get<Vector>(value);
}

const Entity& ActionParameter::getEntityValue(ParamValue value)
{
    return get<Entity>(value);
}

const FuzzyIntervalInt& ActionParameter::getFuzzyIntervalIntValue(ParamValue value)
{
    return get<FuzzyIntervalInt>(value);
}

const FuzzyIntervalFloat& ActionParameter::getFuzzyIntervalFloatValue(ParamValue value)
{
    return get<FuzzyIntervalFloat>(value);
}


bool ActionParameter::areFromSameType(const ParamValue& v1, const ParamValue& v2)
{
    bool result = boost::apply_visitor(same_type_visitor(), v1, v2);
    return result;
}

std::string ActionParameter::stringRepresentation() const throw (opencog::RuntimeException, std::bad_exception)
{

    std::string answer;

    ActionParamTypeCode typeCode = type.getCode();
    switch (typeCode) {
    // bool,int and float are stored as string as well
    case BOOLEAN_CODE:
    case INT_CODE:
    case FLOAT_CODE:
    case STRING_CODE: {
        answer = boost::get<std::string>(value);
        break;
    }
    case VECTOR_CODE: {
        answer = "(";
        answer.append(opencog::toString(boost::get<Vector>(value).x));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Vector>(value).y));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Vector>(value).z));
        answer.append(")");
        break;
    }
    case ROTATION_CODE: {
        answer = "(";
        answer.append(opencog::toString(boost::get<Rotation>(value).pitch));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Rotation>(value).roll));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Rotation>(value).yaw));
        answer.append(")");
        break;
    }
    case ENTITY_CODE: {
        answer = "(";
        answer.append(opencog::toString(boost::get<Entity>(value).id));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Entity>(value).type));
        answer.append(")");
        break;
    }
    case FUZZY_INTERVAL_INT_CODE: {
        answer = "(";
        answer.append(opencog::toString(boost::get<FuzzyIntervalInt>(value).bound_low));
        answer.append(",");
        answer.append(opencog::toString(boost::get<FuzzyIntervalInt>(value).bound_high));
        answer.append(")");
        break;
    }
    case FUZZY_INTERVAL_FLOAT_CODE: {
        answer = "(";
        answer.append(opencog::toString(boost::get<FuzzyIntervalFloat>(value).bound_low));
        answer.append(",");
        answer.append(opencog::toString(boost::get<FuzzyIntervalFloat>(value).bound_high));
        answer.append(")");
        break;
    }
    default: {
        throw opencog::RuntimeException(TRACE_INFO,
                                        "ActionParameter - Invalid param type: %d", typeCode);
        break;
    }
    }

    return answer;
}



std::string ActionParameter::ParamValueToString(const ParamValue& paramVal)
{
    std::string answer ="";

    if(boost::get<std::string>(&(paramVal)))
    {
        answer = boost::get<std::string>(paramVal);

    }
    else if(boost::get<Entity>(&(paramVal)))
    {
        Entity e = boost::get<Entity>(paramVal);
        answer = "(";
        answer.append(opencog::toString(boost::get<Entity>(paramVal).id));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Entity>(paramVal).type));
        answer.append(")");
    }
    else if(boost::get<Vector>(&(paramVal)))
    {
        answer = "(";
        answer.append(opencog::toString(boost::get<Vector>(paramVal).x));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Vector>(paramVal).y));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Vector>(paramVal).z));
        answer.append(")");
    }
    else if(boost::get<Rotation>(&(paramVal)))
    {
        answer = "(";
        answer.append(opencog::toString(boost::get<Rotation>(paramVal).pitch));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Rotation>(paramVal).roll));
        answer.append(",");
        answer.append(opencog::toString(boost::get<Rotation>(paramVal).yaw));
        answer.append(")");
    }
    else if(boost::get<FuzzyIntervalInt>(&(paramVal)))
    {
        answer = "(";
        answer.append(opencog::toString(boost::get<FuzzyIntervalInt>(paramVal).bound_low));
        answer.append(",");
        answer.append(opencog::toString(boost::get<FuzzyIntervalInt>(paramVal).bound_high));
        answer.append(")");
    }
    else if(boost::get<FuzzyIntervalFloat>(&(paramVal)))
    {
        answer = "(";
        answer.append(opencog::toString(boost::get<FuzzyIntervalFloat>(paramVal).bound_low));
        answer.append(",");
        answer.append(opencog::toString(boost::get<FuzzyIntervalFloat>(paramVal).bound_high));
        answer.append(")");
    }

    return answer;

}
