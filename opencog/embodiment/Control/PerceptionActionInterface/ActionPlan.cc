/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionPlan.cc
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


#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>
#include <opencog/util/StringManipulator.h>

#include "ActionPlan.h"
#include "PAIUtils.h"
#include "PVPXmlConstants.h"

#include <iostream>
#include <vector>

using namespace opencog::pai;


ActionPlan::ActionPlan()
{
    PAIUtils::initializeXMLPlatform();
    this->id = opencog::toString(ULONG_MAX); // invalid id.
}

ActionPlan::ActionPlan(ActionPlanID id)
{
    PAIUtils::initializeXMLPlatform();
    this->id = id;
}

ActionPlanID ActionPlan::getID() const
{
    return id;
}

// TODO: Add an option "parallel" boolean argument to this method to indicate if this action
//       should be executed in parallel with the previous or next actions also marked as parallel (default value should be false).
//       If this argument is true, a "parallel" xml element must encapsulate all consecutive
//       actions added to this action plan and marked as parallel, as follows:
//   <action-plan id="0" ...
//       <parallel>
//           <action name="bark" sequence="1"/>
//           <action name="walk" sequence="2">
//               <param name="...
//           </action>
//       </parallel>
//       <action name="grab" sequence="3"/>
//       ...
//   </action-plan>
//
unsigned int ActionPlan::addAction(const PetAction& action) throw (opencog::InvalidParamException, std::bad_exception)
{

    if (!action.containsValidParameters()) {
        throw opencog::InvalidParamException(TRACE_INFO,
                                             "ActionPlan - PetAction with invalid parameters: '%s'", action.getName().c_str());
    }

    PetAction internalAction = action;
    internalAction.setSequence(actions.size() + 1);
    actions.push_back(internalAction);
    actionTried.push_back(false);
    actionDone.push_back(false);
    actionFailed.push_back(false);
    return actions.size();
}

bool ActionPlan::isSeqNumberValid(unsigned int seqNumber) const
{
    return (seqNumber > 0 && seqNumber <= actions.size());
}

const PetAction& ActionPlan::getAction(unsigned int seqNumber) const throw (opencog::IndexErrorException, std::bad_exception)
{
    //assert(isSeqNumberValid(seqNumber));
    if (!isSeqNumberValid(seqNumber)) {
        throw opencog::IndexErrorException(TRACE_INFO,
                                           "ActionPlan - Invalid action sequence number: '%d'.", seqNumber);
    }
    return actions[seqNumber-1];
}

bool ActionPlan::empty() const
{
    return actions.empty();
}

unsigned int ActionPlan::size() const
{
    return actions.size();
}

string ActionPlan::getPVPmessage(const std::string& petId) const
{

    XERCES_CPP_NAMESPACE::DOMDocument* xmlDoc = createEmbodimentXMLDocument();

    XERCES_CPP_NAMESPACE::DOMElement* actionPlanElem = createActionPlanElement(xmlDoc, xmlDoc->getDocumentElement(), petId);

    for (vector<PetAction>::const_iterator itr = actions.begin(); itr != actions.end(); itr++) {
        itr->createPVPXmlElement(xmlDoc, actionPlanElem);
    }
    string result = PAIUtils::getSerializedXMLString(xmlDoc);

    // free memory of the DomDocument
    xmlDoc->release();

    return result;
}

string ActionPlan::getPVPmessage(const std::string& petId, unsigned int actionSeqNum) const
{
    XERCES_CPP_NAMESPACE::DOMDocument* xmlDoc = createEmbodimentXMLDocument();

    XERCES_CPP_NAMESPACE::DOMElement* actionPlanElem = createActionPlanElement(xmlDoc, xmlDoc->getDocumentElement(), petId);

	const PetAction& action = getAction(actionSeqNum);
	action.createPVPXmlElement(xmlDoc, actionPlanElem);

	string result = PAIUtils::getSerializedXMLString(xmlDoc);

    // free memory of the DomDocument
    xmlDoc->release();

    return result;
}

bool ActionPlan::isTried(unsigned int seqNumber) const
{
    bool result = false;
    if (isSeqNumberValid(seqNumber)) {
        result = actionTried[seqNumber-1];
    }
    return result;
}

bool ActionPlan::isDone(unsigned int seqNumber) const
{
    bool result = false;
    if (isSeqNumberValid(seqNumber)) {
        result = actionDone[seqNumber-1];
    }
    return result;
}

bool ActionPlan::isFailed(unsigned int seqNumber) const
{
    bool result = false;
    if (isSeqNumberValid(seqNumber)) {
        result = actionFailed[seqNumber-1];
    }
    return result;
}

void ActionPlan::markAsTried(unsigned int seqNumber)
{
    if (isSeqNumberValid(seqNumber)) {
        actionTried[seqNumber-1] = true;
    }
}

void ActionPlan::markAsDone(unsigned int seqNumber)
{
    if (isSeqNumberValid(seqNumber)) {
        actionDone[seqNumber-1] = true;
    }
}

void ActionPlan::markAsFailed(unsigned int seqNumber)
{
    if (isSeqNumberValid(seqNumber)) {
        actionFailed[seqNumber-1] = true;
    }
}

bool ActionPlan::isFinished() const
{
    bool result = true;
    for (unsigned int i = 0; i < actionDone.size(); i++) {
        if (!actionDone[i] && !actionFailed[i]) {
            result = false;
            break;
        }
    }
    return result;
}

bool ActionPlan::hasFailed() const
{
    bool result = false;
    for (unsigned int i = 0; i < actionFailed.size(); i++) {
        if (actionFailed[i]) {
            result = true;
            break;
        }
    }
    return result;
}

/* --------------------------------------------
 * Private methods used to create XML elements
 * --------------------------------------------
 */
XERCES_CPP_NAMESPACE::DOMDocument* ActionPlan::createEmbodimentXMLDocument() const throw (opencog::XMLException, std::bad_exception)
{
    XMLCh namespaceURI[PAIUtils::MAX_TAG_LENGTH+1];
    XMLCh qualifiedName[PAIUtils::MAX_TAG_LENGTH+1];
    XERCES_CPP_NAMESPACE::XMLString::transcode("http://www.opencog.org/brain", namespaceURI, PAIUtils::MAX_TAG_LENGTH);
    XERCES_CPP_NAMESPACE::XMLString::transcode(ACTION_PLAN_ELEMENT, qualifiedName, PAIUtils::MAX_TAG_LENGTH);
    XERCES_CPP_NAMESPACE::DOMDocument* doc = PAIUtils::getDOMImplementation()->createDocument(
                namespaceURI,
                qualifiedName,
                NULL
            );

    if (!doc) {
        throw opencog::XMLException(TRACE_INFO, "ActionPlan - Error creating DOMDocument.");
    }

    XMLCh tmpStr[PAIUtils::MAX_TAG_LENGTH+1];
    XERCES_CPP_NAMESPACE::XMLString::transcode("UTF-8", tmpStr, PAIUtils::MAX_TAG_LENGTH);
    doc->setEncoding(tmpStr);
    XERCES_CPP_NAMESPACE::XMLString::transcode("1.0", tmpStr, PAIUtils::MAX_TAG_LENGTH);
    doc->setVersion(tmpStr);

    return doc;
}

XERCES_CPP_NAMESPACE::DOMElement* ActionPlan::createActionPlanElement(XERCES_CPP_NAMESPACE::DOMDocument* doc,
        XERCES_CPP_NAMESPACE::DOMElement* parent,
        const std::string& petId) const
{

    XERCES_CPP_NAMESPACE::DOMElement *actionPlan = parent;

    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    XERCES_CPP_NAMESPACE::XMLString::transcode(ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLCh* idStr = XERCES_CPP_NAMESPACE::XMLString::transcode(opencog::toString(id).c_str());
    actionPlan->setAttribute(tag, idStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&idStr);

    XERCES_CPP_NAMESPACE::XMLString::transcode(ENTITY_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLCh* entityIdStr = XERCES_CPP_NAMESPACE::XMLString::transcode(PAIUtils::getExternalId(petId.c_str()).c_str());
    actionPlan->setAttribute(tag, entityIdStr);
    XERCES_CPP_NAMESPACE::XMLString::release(&entityIdStr);

    return actionPlan;
}
