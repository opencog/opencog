/*
 * opencog/embodiment/Control/PerceptionActionInterface/AvatarAction.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#include "AvatarAction.h"

#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>
#include <opencog/util/StringManipulator.h>

#include "PAIUtils.h"
#include "PVPXmlConstants.h"

using namespace opencog::pai;
using namespace opencog;

AvatarAction::AvatarAction()
{
    PAIUtils::initializeXMLPlatform();
    type = ActionType::DO_NOTHING().getCode(); // default action. Just to be valid.
    sequence = 0;
}

AvatarAction::AvatarAction(const ActionType& type)
{
    PAIUtils::initializeXMLPlatform();
    this->type = type.getCode();
    this->sequence = 0;
}

AvatarAction::~AvatarAction()
{
}

void AvatarAction::setType(const ActionType& actionType) throw (opencog::InvalidParamException, std::bad_exception)
{
    std::string errorMessage = validateParameters(actionType, parameters);
    if (!errorMessage.empty()) {
        throw opencog::InvalidParamException(TRACE_INFO, "AvatarAction - Invalid parameter: '%s'. %s.",
                                             actionType.getName().c_str(), errorMessage.c_str());
    }
    this->type = actionType.getCode();
}

const ActionType& AvatarAction::getType() const
{
    return ActionType::getFromCode(type);
}

const string& AvatarAction::getName() const
{
    return ActionType::getFromCode(type).getName();
}

void AvatarAction::setSequence(unsigned int sequence)
{
    this->sequence = sequence;
}

unsigned int AvatarAction::getSequence() const
{
    return this->sequence;
}

void AvatarAction::addParameter(ActionParameter param) throw (opencog::InvalidParamException, std::bad_exception)
{

    const ActionType::ParamTypes& mandatoryParamTypes = ActionType::getFromCode(type).getMandatoryParamTypes();
    if (parameters.size() < mandatoryParamTypes.size()) {
        if (param.getType() != mandatoryParamTypes[parameters.size()]) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "AvatarAction - Param '%s', type '%s' with wrong type for parameter '%u' (mandatory: '%s').", param.getName().c_str(), param.getType().getName().c_str(), parameters.size() + 1, mandatoryParamTypes[parameters.size()].getName().c_str());
        }
    } else {
        const ActionType::ParamTypes& optionalParamTypes =  ActionType::getFromCode(type).getOptionalParamTypes();
        unsigned int numberOfOptionalParams = parameters.size() - mandatoryParamTypes.size();
        if (numberOfOptionalParams >= optionalParamTypes.size()) {
            throw opencog::InvalidParamException(TRACE_INFO, "AvatarAction - Param '%s', type '%s': maximum number of parameters already reached", param.getName().c_str(), param.getType().getName().c_str());
        }
        if (param.getType() != optionalParamTypes[numberOfOptionalParams]) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "AvatarAction - Param '%s', type '%s': wrong type for parameter %u (optional: %s)", param.getName().c_str(), param.getType().getName().c_str(), parameters.size() + 1, optionalParamTypes[numberOfOptionalParams].getName().c_str());
        }
    }
    parameters.push_back(param);
}

const list<ActionParameter>& AvatarAction::getParameters() const
{
    return parameters;
}

bool AvatarAction::containsValidParameters() const
{
    std::string errorMessage = validateParameters(ActionType::getFromCode(type), parameters);
    if (errorMessage.size() > 0) {
        logger().warn("AvatarAction::" + errorMessage);
        return false;
    }
    return true;
}

std::string AvatarAction::validateParameters(const ActionType& actionType, const list<ActionParameter>& params)
{
    char buffer[1<<16];

    // Check arity:
    const ActionType::ParamTypes& mandatoryParamTypes = actionType.getMandatoryParamTypes();
    // Check lower bound
    if (params.size() < mandatoryParamTypes.size()) {
        sprintf(buffer, "validateParameters(): Insufficient number of parameters (%zu) for action '%s'", params.size(), actionType.getName().c_str());
        return buffer;
    }
    const ActionType::ParamTypes& optionalParamTypes = actionType.getOptionalParamTypes();
    // Check upper bound
    if (params.size() > (mandatoryParamTypes.size() + optionalParamTypes.size())) {
        sprintf(buffer, "validateParameters(): Exceeded number of parameters (%zu) for action '%s'", params.size(), actionType.getName().c_str());
        return buffer;
    }

    // Check param types
    list<ActionParameter>::const_iterator paramItr = params.begin();

    // Mandatory parameters
    ActionType::ParamTypes::const_iterator paramTypeItr = mandatoryParamTypes.begin();
    while (paramTypeItr != mandatoryParamTypes.end()) {
        if (paramItr->getType() != *paramTypeItr) {
            sprintf(buffer, "validateParameters(): Mandatory parameter type mismatch (param type = '%s', expected type = '%s')", paramItr->getType().getName().c_str(), paramTypeItr->getName().c_str());
            return buffer;
        }
        ++paramTypeItr;
        ++paramItr;
    }

    // Optional parameters
    paramTypeItr = optionalParamTypes.begin();
    while (paramItr != params.end()) {
        if (paramItr->getType() != *paramTypeItr) {
            sprintf(buffer, "AvatarAction::containsValidParameters(): Optional parameter type mismatch (param type = '%s', expected type = '%s')", paramItr->getType().getName().c_str(), paramTypeItr->getName().c_str());
            return buffer;
        }
        ++paramTypeItr;
        ++paramItr;
    }

    return "";
}

XERCES_CPP_NAMESPACE::DOMElement* AvatarAction::createPVPXmlElement(XERCES_CPP_NAMESPACE::DOMDocument* doc,
        XERCES_CPP_NAMESPACE::DOMElement* parent) const
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    XERCES_CPP_NAMESPACE::XMLString::transcode(ACTION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    XERCES_CPP_NAMESPACE::DOMElement *actionElement = doc->createElement(tag);

    XERCES_CPP_NAMESPACE::XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLCh* nameStr = XERCES_CPP_NAMESPACE::XMLString::transcode(ActionType::getFromCode(type).getName().c_str());
    actionElement->setAttribute(tag, nameStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&nameStr);

    XERCES_CPP_NAMESPACE::XMLString::transcode(SEQUENCE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLCh* sequenceStr = XERCES_CPP_NAMESPACE::XMLString::transcode(opencog::toString(sequence).c_str());
    actionElement->setAttribute(tag, sequenceStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&sequenceStr);


    parent->appendChild(actionElement);
    for (list<ActionParameter>::const_iterator itr = parameters.begin(); itr != parameters.end(); ++itr) {
        itr->createPVPXmlElement(doc, actionElement);
    }
    return actionElement;
}

char *AvatarAction::getAttribute(XERCES_CPP_NAMESPACE::DOMElement *element, const char *attributeName)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH + 1];
    XERCES_CPP_NAMESPACE::XMLString::transcode(attributeName, tag, PAIUtils::MAX_TAG_LENGTH);
    // TODO: not sure this is a memory leak. The method transcode() below returns a char *
    // but I'm not sure if caller's supposed to release the memory or if it's done
    // when DOMElement is deleted. If it's a memory leak, this method and the one below should
    // be revised.
    return XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
}

XERCES_CPP_NAMESPACE::DOMNodeList *AvatarAction::getChildren(XERCES_CPP_NAMESPACE::DOMElement *element, const char *tagName)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH + 1];
    XERCES_CPP_NAMESPACE::XMLString::transcode(tagName, tag, PAIUtils::MAX_TAG_LENGTH);
    return element->getElementsByTagName(tag);
}

AvatarAction *AvatarAction::factory(XERCES_CPP_NAMESPACE::DOMElement *actionDOMElement) throw (opencog::InvalidParamException, std::bad_exception)
{

    std::string actionName = getAttribute(actionDOMElement, NAME_ATTRIBUTE);
    ActionType actionType = ActionType::getFromName(actionName);
    AvatarAction *avatarAction = new AvatarAction(actionType);
    avatarAction->setSequence((unsigned int) atoi(getAttribute(actionDOMElement, SEQUENCE_ATTRIBUTE)));

    XERCES_CPP_NAMESPACE::DOMNodeList *list = getChildren(actionDOMElement, PARAMETER_ELEMENT);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        XERCES_CPP_NAMESPACE::DOMElement *parameterDOMElement = (XERCES_CPP_NAMESPACE::DOMElement *) list->item(i);
        std::string parameterName = getAttribute(parameterDOMElement, NAME_ATTRIBUTE);
        ActionParamType parameterType = ActionParamType::getFromName(getAttribute(parameterDOMElement, TYPE_ATTRIBUTE));
        ActionParamTypeCode typeCode = parameterType.getCode();
        ParamValue parameterValue;
        switch (typeCode) {
            // boolean, int and float are used as strings
        case BOOLEAN_CODE:
        case INT_CODE:
        case FLOAT_CODE:
        case STRING_CODE: {
            std::string stringValue = getAttribute(parameterDOMElement, VALUE_ATTRIBUTE);
            parameterValue = stringValue;
            break;
        }
        case VECTOR_CODE: {
            XERCES_CPP_NAMESPACE::DOMElement *vectorDOMElement = (XERCES_CPP_NAMESPACE::DOMElement *) getChildren(parameterDOMElement, VECTOR_ELEMENT)->item(0);
            double x = atof(getAttribute(vectorDOMElement, X_ATTRIBUTE));
            double y = atof(getAttribute(vectorDOMElement, Y_ATTRIBUTE));
            double z = atof(getAttribute(vectorDOMElement, Z_ATTRIBUTE));
            Vector vectorValue(x, y, z);
            parameterValue = vectorValue;
            break;
        }
        case ROTATION_CODE: {
            XERCES_CPP_NAMESPACE::DOMElement *rotationDOMElement = (XERCES_CPP_NAMESPACE::DOMElement *) getChildren(parameterDOMElement, ROTATION_ELEMENT)->item(0);
            double pitch = atof(getAttribute(rotationDOMElement, PITCH_ATTRIBUTE));
            double roll = atof(getAttribute(rotationDOMElement, ROLL_ATTRIBUTE));
            double yaw = atof(getAttribute(rotationDOMElement, YAW_ATTRIBUTE));
            Rotation rotationValue(pitch, roll, yaw);
            parameterValue = rotationValue;
            break;
        }
        case ENTITY_CODE: {
            XERCES_CPP_NAMESPACE::DOMElement *entityDOMElement = (XERCES_CPP_NAMESPACE::DOMElement *) getChildren(parameterDOMElement, ENTITY_ELEMENT)->item(0);
//                std::string id = PAIUtils::getInternalId(getAttribute(entityDOMElement, ID_ATTRIBUTE));
            std::string id = getAttribute(entityDOMElement, ID_ATTRIBUTE);
            std::string type = getAttribute(entityDOMElement, TYPE_ATTRIBUTE);
            Entity entityValue(id, type);
            parameterValue = entityValue;
            break;
        }
        default: {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "AvatarAction - Invalid param type code: %d", typeCode);
            break;
        }
        }
        ActionParameter actionParameter(parameterName, parameterType, parameterValue);
        avatarAction->addParameter(actionParameter);
    }

    return avatarAction;
}

std::string AvatarAction::stringRepresentation() const
{

    char s[16];
    sprintf(s, "%d:", getSequence());
    std::string answer = s;
    answer.append(getName());
    answer.append("(");
    unsigned int count = 1;
    for (list<ActionParameter>::const_iterator it = parameters.begin(); it != parameters.end(); ++it) {
        try {
            answer.append(it->stringRepresentation());
            if (count != parameters.size()) {
                answer.append(",");
            }
            count++;
        } catch (opencog::RuntimeException& e) {
            // just ignore the string representation of the ActionParameter that
            // caused the exception
        }
    }
    answer.append(")");

    return answer;
}

