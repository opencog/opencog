/*
 * opencog/embodiment/Control/PerceptionActionInterface/PAI.cc
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
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

#include <xercesc/dom/DOM.hpp>
#include <xercesc/framework/MemBufInputSource.hpp>

#include <xercesc/framework/Wrapper4InputSource.hpp>
#include <xercesc/validators/common/Grammar.hpp>
#include <xercesc/validators/schema/SchemaGrammar.hpp>
#include <xercesc/util/XMLUni.hpp>

#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include <iostream>
#include <cstring>
#include <cstdlib>

#include <boost/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/Logger.h>
#include <opencog/util/files.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>

#include "PAI.h"
#include "PAIUtils.h"
#include "PVPXmlConstants.h"
#include "EmbodimentDOMParser.h"
#include "EmbodimentErrorHandler.h"

using XERCES_CPP_NAMESPACE::XMLString;
using XERCES_CPP_NAMESPACE::DOMDocument;
using XERCES_CPP_NAMESPACE::DOMElement;
using XERCES_CPP_NAMESPACE::DOMNode;
using XERCES_CPP_NAMESPACE::DOMNodeList;

using namespace boost::posix_time;
using namespace boost::gregorian;

using namespace opencog::pai;
using namespace opencog::control;
using namespace opencog;

PAI::PAI(AtomSpace& _atomSpace, ActionPlanSender& _actionSender,
        AvatarInterface& _avatarInterface, unsigned long nextPlanID) :
    atomSpace(_atomSpace), actionSender(_actionSender),
    avatarInterface(_avatarInterface), nextActionPlanId(nextPlanID)
{
    PAIUtils::initializeXMLPlatform();
    xMin = -1;
    yMin = -1;
    xMax = -1;
    yMax = -1;
    agentRadius = -1;
    agentHeight = -1;
    floorHeight = 0;

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_init(&plock, NULL);
#endif
    latestSimWorldTimestamp = 0;

    // Set up the xml parser
    parser = new EmbodimentDOMParser();
    parser->setErrorHandler(&errorHandler);

    // The following lines enable validation
    //parser->setValidationScheme(XERCES_CPP_NAMESPACE::XercesDOMParser::Val_Always);
    parser->cacheGrammarFromParse(true);
    // only validate when setDoSchema(true) is called.
    parser->setValidationScheme(XERCES_CPP_NAMESPACE::XercesDOMParser::Val_Auto);
    parser->setDoNamespaces(true);
//    parser->setDoNamespaces(false);
    parser->setDoSchema(true);
#if 1 // Add the XSD file to be used for xml validation   
#define XSD_NAMESPACE "http://www.opencog.org/brain"
#define XSD_FILE_NAME "BrainProxyAxon.xsd"

    if (fileExists(XSD_FILE_NAME))  {
        schemaLocation += XSD_NAMESPACE " " XSD_FILE_NAME;
    } else if (fileExists(PVP_XSD_FILE_PATH)) {
        schemaLocation += XSD_NAMESPACE " " PVP_XSD_FILE_PATH;
    }
    if (schemaLocation.size() > 0) {
        logger().info("PAI - Setting the schemaLocation to: %s\n", schemaLocation.c_str());
        // The line bellow replace the path for the XSD file so that it does not need to be in the current directory...
        parser->setExternalSchemaLocation(schemaLocation.c_str());
    }
#endif

    logPVPMessage = !(config().get_bool("DISABLE_LOG_OF_PVP_MESSAGES"));

}

PAI::~PAI()
{
    delete parser;
    // TODO: Cannot terminate here because other PAI objects may be using it...
    //PAIUtils::terminateXMLPlatform();
}

AtomSpace& PAI::getAtomSpace()
{
    return atomSpace;
}

AvatarInterface& PAI::getAvatarInterface()
{
    return avatarInterface;
}

ActionPlanID PAI::createActionPlan()
{
    ActionPlanID planId;
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_lock(&plock);
#endif
    planId = opencog::toString(nextActionPlanId);
    nextActionPlanId++;
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_unlock(&plock);
#endif
    ActionPlan plan(planId);
    inProgressActionPlans[planId] = plan;
    ActionIdMap actionIdMap;
    planToActionIdsMaps[planId] = actionIdMap;
    return planId;
}

void PAI::sendActionPlan(ActionPlanID planId) throw (opencog::RuntimeException, std::bad_exception)
{
    ActionPlanMap::const_iterator it = inProgressActionPlans.find(planId);
    if (it != inProgressActionPlans.end()) {
        const ActionPlan& plan = it->second;
        if (actionSender.sendActionPlan(plan)) {
            // mark action plan as sent by moving it from inProgress to pending map
            
            // must be added first. Otherwise the reference to the plan becomes invalid
            pendingActionPlans[planId] = plan;
            inProgressActionPlans.erase(it->first);

            // TODO: Add a "ActionTried" predicate for each action in the sent action plan
            // (i.e., each ExecLink Handle in planToActionIdsMaps[planId])
        } else {
            throw opencog::RuntimeException(TRACE_INFO,
                "PAI - ActionPlanSender could not send the ActionPlan '%s'.", planId.c_str());
        }
    } else {
        throw opencog::RuntimeException(TRACE_INFO,
            "PAI - No ActionPlan with the id '%s'.", planId.c_str());
    }
}

void PAI::sendExtractedActionFromPlan(ActionPlanID planId, unsigned int actionSeqNum) throw (opencog::RuntimeException, std::bad_exception)
{
    ActionPlanMap::const_iterator it = inProgressActionPlans.find(planId);

	// send the first action in the plan and mark the plan as a pending one.
    if (it != inProgressActionPlans.end()) {
        const ActionPlan& plan = it->second;
        if (actionSender.sendSpecificActionFromPlan(plan, actionSeqNum)) {
            // mark action plan as sent by moving it from inProgress to pending map
            
            // must be added first. Otherwise the reference to the plan becomes invalid
            pendingActionPlans[planId] = plan;
            inProgressActionPlans.erase(it->first);

        } else {
            throw opencog::RuntimeException(TRACE_INFO,
                "PAI - ActionPlanSender could not send the ActionPlan '%s'.", planId.c_str());
        }
    } else {
		// The plan is not in the progress action plan list. Then check
		// if the plan is already in the pending list or not.
		it = pendingActionPlans.find(planId);
		if (it != pendingActionPlans.end()) {
			const ActionPlan& plan = it->second;
			if (!actionSender.sendSpecificActionFromPlan(plan, actionSeqNum)) {
				throw opencog::RuntimeException(TRACE_INFO,
					"PAI - ActionPlanSender could not send the ActionPlan '%s'.", planId.c_str());
			}
		} else {
            throw opencog::RuntimeException(TRACE_INFO,
                "PAI - ActionPlanSender could not send the ActionPlan '%s'.", planId.c_str());
        }

    }
}

void PAI::sendEmotionalFeelings(const std::string& petId, const std::map<std::string, float>& feelingsValueMap)
{

    if (feelingsValueMap.empty()) return;

    // creating XML DOC
    XMLCh namespaceURI[PAIUtils::MAX_TAG_LENGTH+1];
    XMLCh qualifiedName[PAIUtils::MAX_TAG_LENGTH+1];
    XMLString::transcode("http://www.opencog.org/brain", namespaceURI, PAIUtils::MAX_TAG_LENGTH);
    XMLString::transcode(EMOTIONAL_FEELING_ELEMENT, qualifiedName, PAIUtils::MAX_TAG_LENGTH);
    DOMDocument* doc = PAIUtils::getDOMImplementation()->createDocument(
                namespaceURI,
                qualifiedName,
                NULL
            );

    if (!doc) {
        throw opencog::XMLException(TRACE_INFO, "ActionPlan - Error creating DOMDocument.");
    }

    XMLCh tmpStr[PAIUtils::MAX_TAG_LENGTH+1];
    // Set encoding
    XMLString::transcode("UTF-8", tmpStr, PAIUtils::MAX_TAG_LENGTH);
    doc->setEncoding(tmpStr);
    // Set version
    XMLString::transcode("1.0", tmpStr, PAIUtils::MAX_TAG_LENGTH);
    doc->setVersion(tmpStr);

    // filling emotional feeling element with pet id
    DOMElement *emotionalFeeling = doc->getDocumentElement();

    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    XMLString::transcode(ENTITY_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLCh* entityIdStr = XMLString::transcode(PAIUtils::getExternalId(petId.c_str()).c_str());
    emotionalFeeling->setAttribute(tag, entityIdStr);
    XMLString::release(&entityIdStr);

    // adding all feelings
    std::map<std::string, float>::const_iterator it;
    for (it = feelingsValueMap.begin(); it != feelingsValueMap.end(); it++) {

        XMLString::transcode(FEELING_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        DOMElement *feelingElement = doc->createElement(tag);

        XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* nameStr = XMLString::transcode(it->first.c_str());
        feelingElement->setAttribute(tag, nameStr);
        XMLString::release(&nameStr);

        XMLString::transcode(VALUE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* valueStr = XMLString::transcode(opencog::toString(it->second).c_str());
        feelingElement->setAttribute(tag, valueStr);
        XMLString::release(&valueStr);

        emotionalFeeling->appendChild(feelingElement);
    }

    // sending message
    string result = PAIUtils::getSerializedXMLString(doc);
    actionSender.sendEmotionalFeelings(result);

    // free memory of the DomDocument
    doc->release();
}

void PAI::sendDemandSatisfactions(const std::string& petId, const std::map<std::string, float> & demandsValueMap)
{
    if (demandsValueMap.empty()) return;

    // creating XML DOC
    XMLCh namespaceURI[PAIUtils::MAX_TAG_LENGTH+1];
    XMLCh qualifiedName[PAIUtils::MAX_TAG_LENGTH+1];
    XMLString::transcode("http://www.opencog.org/brain", namespaceURI, PAIUtils::MAX_TAG_LENGTH);
    XMLString::transcode(PSI_DEMAND_ELEMENT, qualifiedName, PAIUtils::MAX_TAG_LENGTH);
    DOMDocument* doc = PAIUtils::getDOMImplementation()->createDocument(namespaceURI,
                                                                        qualifiedName,
                                                                        NULL
                                                                        );

    if (!doc) {
        throw opencog::XMLException(TRACE_INFO, "ActionPlan - Error creating DOMDocument.");
    }

    XMLCh tmpStr[PAIUtils::MAX_TAG_LENGTH+1];
    // set encoding
    XMLString::transcode("UTF-8", tmpStr, PAIUtils::MAX_TAG_LENGTH);
    doc->setEncoding(tmpStr);
    // set version
    XMLString::transcode("1.0", tmpStr, PAIUtils::MAX_TAG_LENGTH);
    doc->setVersion(tmpStr);

    // filling emotional feeling element with pet id
    DOMElement * demandSatisfaction = doc->getDocumentElement();

    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    XMLString::transcode(ENTITY_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLCh* entityIdStr = XMLString::transcode(PAIUtils::getExternalId(petId.c_str()).c_str());
    demandSatisfaction->setAttribute(tag, entityIdStr);
    XMLString::release(&entityIdStr);

    // adding all the demand satisfactions
    std::map<std::string, float>::const_iterator it;
    for (it = demandsValueMap.begin(); it != demandsValueMap.end(); it++) {

        XMLString::transcode(DEMAND_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        DOMElement * demandElement = doc->createElement(tag);

        XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* nameStr = XMLString::transcode(it->first.c_str());
        demandElement->setAttribute(tag, nameStr);
        XMLString::release(&nameStr);

        XMLString::transcode(VALUE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        XMLCh* valueStr = XMLString::transcode(opencog::toString(it->second).c_str());
        demandElement->setAttribute(tag, valueStr);
        XMLString::release(&valueStr);

        demandSatisfaction->appendChild(demandElement);
    }

    // sending message
    string result = PAIUtils::getSerializedXMLString(doc);
    actionSender.sendEmotionalFeelings(result);

    // free memory of the DomDocument
    doc->release();
}

ActionID PAI::addAction(ActionPlanID planId, const PetAction& action) throw (opencog::RuntimeException, opencog::InvalidParamException, std::bad_exception)
{
    ActionID result = Handle::UNDEFINED;
    ActionPlanMap::iterator it = inProgressActionPlans.find(planId);

    if (it != inProgressActionPlans.end()) {
        ActionPlan& plan = it->second;

        // This function can throw an InvalidParamException
        unsigned int seqNumber = plan.addAction(action);
        result = addActionToAtomSpace(planId, action);

        // Maps the planID + seqNumber to the actionID for further usage.
        planToActionIdsMaps[planId][seqNumber] = result;
    } else {
        logger().warn("PAI - No action plan with id = %s in progress\n", planId.c_str());
        // TODO: throw an Exception?
    }
    return result;
}

bool PAI::isActionPlanEmpty(const ActionPlanID& planId)
{
    ActionPlanMap::iterator it = inProgressActionPlans.find(planId);

    if (it != inProgressActionPlans.end()) {
        return it->second.empty();
    } else {
        logger().warn(
                     "PAI - No action plan with id = %s in progress\n",
                     planId.c_str());
        return false;
    }
}

bool PAI::processPVPMessage(const string& pvpMsg, HandleSeq &toUpdateHandles)
{
    logger().debug("PAI - processPVPMessage atomSpace.size=%d", atomSpace.getSize() );

    if (logPVPMessage) {
        logger().info(" PAI - Processing PVP message:\n%s\n", pvpMsg.c_str());
    }

    static const char* bufID = "pvp message";
    const XMLByte* xmlBuf = reinterpret_cast<const XMLByte*>(pvpMsg.c_str());
    XERCES_CPP_NAMESPACE::MemBufInputSource * memBufIS =
        new XERCES_CPP_NAMESPACE::MemBufInputSource((const XMLByte *) xmlBuf, pvpMsg.size(), bufID);

    parser->resetDocumentPool();

    try {
        parser->parse(*memBufIS);
    } catch (const XERCES_CPP_NAMESPACE::XMLException& toCatch) {
        char* message = XMLString::transcode(toCatch.getMessage());
        logger().error("PAI - XML Exception: %s\n", message);
        XMLString::release(&message);
        delete memBufIS;
        return false;
    } catch (const XERCES_CPP_NAMESPACE::DOMException& toCatch) {
        char* message = XMLString::transcode(toCatch.msg);
        logger().error("PAI - DOM Exception: %s\n", message);
        XMLString::release(&message);
        delete memBufIS;
        return false;
    } catch (...) {
        logger().error("PAI - Unexpected XML Parse Exception\n");
        delete memBufIS;
        return false;
    }

    DOMDocument * document = NULL;
    if (parser->getErrorCount() == 0) {
        document = parser->adoptDocument();
        logger().debug("PAI - DOMDocument retrieved");

        try {
            processPVPDocument(document, toUpdateHandles);
            logger().debug("PAI - processPVPDocument done");

            // catch runtime exceptions and its specialization
        } catch (opencog::RuntimeException& e) {
            delete memBufIS;
            delete document;
            return false;

        } catch (const std::exception& e) {
            logger().error("PAI - Got an std::exception while processing from "
                    "PVP XML message: %s", e.what( ) );

            delete memBufIS;
            delete document;
            return false;

        } catch (...) {
            logger().error("PAI - Got an unknown exception while processing from "
                    "PVP XML message.");
            delete memBufIS;
            delete document;
            return false;
        }
    } else {
        logger().error("PAI - Got %d errors while parsing xml message data.",
                parser->getErrorCount());
        delete memBufIS;
        delete document;
        return false;
    }

    delete memBufIS;
    //delete parser;
    delete document;

    return true;
}

// TODO: TEMPORARY PUBLIC METHODS: They should become built-in predicates later.
bool PAI::isActionDone(ActionID actionId, unsigned long sinceTimestamp) const
{
    return AtomSpaceUtil::isActionPredicatePresent(
            atomSpace, ACTION_DONE_PREDICATE_NAME, actionId, sinceTimestamp);
}

bool PAI::isActionFailed(ActionID actionId, unsigned long sinceTimestamp) const
{
    return AtomSpaceUtil::isActionPredicatePresent(
            atomSpace, ACTION_FAILED_PREDICATE_NAME, actionId, sinceTimestamp);
}

bool PAI::isActionTried(ActionID actionId, unsigned long sinceTimestamp) const
{
    return AtomSpaceUtil::isActionPredicatePresent(
            atomSpace, ACTION_TRIED_PREDICATE_NAME, actionId, sinceTimestamp);
}

bool PAI::isPlanFinished(ActionPlanID planId) const
{
    return (atof(planId.c_str()) < nextActionPlanId) &&
           (inProgressActionPlans.find(planId) == inProgressActionPlans.end()) &&
           (pendingActionPlans.find(planId) == pendingActionPlans.end());
}

bool PAI::hasPlanFailed(ActionPlanID planId) const
{
    return failedActionPlans.find(planId) != failedActionPlans.end();
}


/* ----------------------------------------------
 * Private methods used to parse XML msg from PVP
 * ----------------------------------------------
 */
void PAI::processPVPDocument(DOMDocument * doc, HandleSeq &toUpdateHandles)
{
    logger().debug("PAI - processPVPDocument");
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];
    DOMNodeList * list;

    // TODO: Check if there is a specific order of XML elements that should be followed
    // For now, MapInfo is processed first since it's supposed to inform SL object types.
    // And Instructions are processed later so that all relevant perceptions
    // are already processed when the owner asks for anything...

    // getting <map-info> elements from the XML message
    XMLString::transcode(MAP_INFO_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    list = doc->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        processMapInfo((DOMElement *)list->item(i), toUpdateHandles);
    }
    if (list->getLength() > 0)
        logger().debug("PAI - Processing %d map-infos done", list->getLength());

    // getting <agent-signal> elements from the XML message
    XMLString::transcode(AGENT_SIGNAL_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    list = doc->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        processAgentSignal((DOMElement *)list->item(i));
    }
    if (list->getLength() > 0)
        logger().debug("PAI - Processing %d agent-signals done", list->getLength());

    // getting <temp-signal> elements from the XML message
    XMLString::transcode(TEMP_SIGNAL_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    list = doc->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        processTempSignal((DOMElement *)list->item(i));
    }
    if (list->getLength() > 0)
        logger().debug("PAI - Processing %d temp-signal done", list->getLength());

    // getting <instructions> elements from the XML message
    XMLString::transcode(INSTRUCTION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    list = doc->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        processInstruction((DOMElement *)list->item(i));
    }
    if (list->getLength() > 0)
        logger().debug("PAI - Processing %d instructions done", list->getLength());

    // getting <agent-sensor-info> elements from the XML message
    XMLString::transcode(AGENT_SENSOR_INFO_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    list = doc->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        processAgentSensorInfo((DOMElement *)list->item(i));
    } // for
    if (list->getLength() > 0)
        logger().debug("PAI - Processing %d agent-sensor-infos done", list->getLength());
}

void PAI::processAgentSignal(DOMElement * element)
    throw (opencog::RuntimeException, opencog::InvalidParamException, std::bad_exception)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    unsigned long tsValue = getTimestampFromElement(element);
    if (!setLatestSimWorldTimestamp(tsValue)) {
        logger().error("PAI - Received old timestamp in agent-signal => Message discarded!");
        return;
    }

    // getting agent-id attribute value
    XMLString::transcode(AGENT_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* agentID = XMLString::transcode(element->getAttribute(tag));
    string internalAgentId = PAIUtils::getInternalId(agentID);

    // getting agent type
    XMLString::transcode(AGENT_TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* agentType = XMLString::transcode(element->getAttribute(tag));
    string agentTypeStr( agentType );

    // add/get the appropriate handle representing this agent, whatever it's
    // type.
    Handle agentNode = processAgentType(agentTypeStr,internalAgentId);

    DOMNodeList * signals = element->getChildNodes();
    for (unsigned int i = 0; i < signals->getLength(); i++) {
        DOMNode* n = signals->item(i);
        // We only deal with actual XML elements
        if (n->getNodeType() != DOMNode::ELEMENT_NODE) continue;
        DOMElement* signal = (DOMElement*) n;
        
        // Both physiological signals and actions have the name attribute
        XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* name = XMLString::transcode(signal->getAttribute(tag));

        logger().debug("PAI - Got agent-signal: agentId = %s (%s), timestamp = %u\n", agentID, internalAgentId.c_str(), tsValue);

        char* signalName = XMLString::transcode(signal->getTagName());
        if (strcmp(signalName,ACTION_ELEMENT) == 0) {
            string nameStr(camelCaseToUnderscore(name));

            // get the plan-id, if any
            XMLString::transcode(ACTION_PLAN_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
            const XMLCh* planID_xml = signal->getAttribute(tag);
            char* planIdStr = XMLString::transcode(planID_xml);
            // if there is a plan-id, this mean the action is a summary of the
            // result without parameters, otherwise it is information about an
            // action by itself.
            if (strlen(planIdStr)) {
                processAgentActionPlanResult(agentID, tsValue, nameStr, planIdStr, signal);
                XMLString::release(&planIdStr);
            } else {
                processAgentActionWithParameters(agentNode, internalAgentId, tsValue, nameStr, signal);
            }

        } else if (strcmp(signalName,PHYSIOLOGY_LEVEL_ELEMENT) == 0) {
            // For physiological signals
            HandleSeq feelingParams;

            XMLString::transcode(VALUE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
            char* levelString = XMLString::transcode(signal->getAttribute(tag));
            float level = boost::lexical_cast<float>(levelString);
            XMLString::release(&levelString);

            logger().fine("physiology level %s at value %.3f", name, level);

            addPhysiologicalFeeling(internalAgentId, name, tsValue, level);

        } 
        XMLString::release(&signalName);
        XMLString::release(&name);
    }

    XMLString::release(&agentID);
    XMLString::release(&agentType);
}

Handle PAI::processAgentActionParameter(DOMElement* paramElement)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];
    Handle resultHandle;

    XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* paramName = XMLString::transcode(paramElement->getAttribute(tag));

    XMLString::transcode(TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* paramType = XMLString::transcode(paramElement->getAttribute(tag));

    XMLString::transcode(VALUE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* paramValue = XMLString::transcode(paramElement->getAttribute(tag));

    logger().debug("PAI - Got param: name = %s, type = %s, value = %s", paramName, paramType, paramValue);

    // This function can throw an InvalidParamException
    const ActionParamType& actionParamType = ActionParamType::getFromName(paramType);

    switch (actionParamType.getCode()) {
    case BOOLEAN_CODE: {
        if (strcmp(paramValue, "true") && strcmp(paramValue, "false")) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "PAI - Invalid value for boolean parameter: '%s'. It should be true or false.", paramValue);
        }
        resultHandle = AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, paramValue);
        break;
    }
    case INT_CODE:
    case FLOAT_CODE: {
        // TODO: check if string is a valid number?
        resultHandle = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, paramValue);
        break;
    }
    case STRING_CODE: {
        // TODO: convert the string to lowercase?
        resultHandle = AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, paramValue);
        break;
    }
    case VECTOR_CODE: {
        XMLString::transcode(VECTOR_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        DOMNodeList * vectorList = paramElement->getElementsByTagName(tag);
        if (vectorList->getLength() <= 0) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "PAI - Got a parameter element with vector type without a vector child element.");
        }
        DOMElement* vectorElement = (DOMElement*) vectorList->item(0);
        XMLString::transcode(X_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* xStr = XMLString::transcode(vectorElement->getAttribute(tag));
        XMLString::transcode(Y_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* yStr = XMLString::transcode(vectorElement->getAttribute(tag));
        XMLString::transcode(Z_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* zStr = XMLString::transcode(vectorElement->getAttribute(tag));
        logger().debug("PAI - Got vector: x = %s, y = %s, z = %s\n", xStr, yStr, zStr);
        HandleSeq rotationListLink;

        rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, xStr));
        rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, yStr));
        rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, zStr));
        resultHandle = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, rotationListLink);

        XMLString::release(&xStr);
        XMLString::release(&yStr);
        XMLString::release(&zStr);
        break;
    }
    case ROTATION_CODE: {
        XMLString::transcode(ROTATION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        DOMNodeList * rotationList = paramElement->getElementsByTagName(tag);
        if (rotationList->getLength() <= 0) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "PAI - Got a parameter element with Rotation type without a rotation child element.");
        }
        DOMElement* rotationElement = (DOMElement*) rotationList->item(0);
        XMLString::transcode(PITCH_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* pitchStr = XMLString::transcode(rotationElement->getAttribute(tag));
        XMLString::transcode(ROLL_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* rollStr = XMLString::transcode(rotationElement->getAttribute(tag));
        XMLString::transcode(YAW_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* yawStr = XMLString::transcode(rotationElement->getAttribute(tag));
        logger().debug("PAI - Got rotaion: pitch = %s, roll = %s, yaw = %s\n", pitchStr, rollStr, yawStr);

        HandleSeq rotationListLink;

        rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, pitchStr));
        rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, rollStr));
        rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, yawStr));
        resultHandle = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, rotationListLink);

        XMLString::release(&pitchStr);
        XMLString::release(&rollStr);
        XMLString::release(&yawStr);
        break;
    }
    case ENTITY_CODE: {
        XMLString::transcode(ENTITY_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        DOMNodeList * entityList = paramElement->getElementsByTagName(tag);
        if (entityList->getLength() <= 0) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "PAI - Got a parameter element with Entity type without an entity child element.");
        }
        DOMElement* entityElement = (DOMElement*) entityList->item(0);

        XMLString::transcode(ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* entityId = XMLString::transcode(entityElement->getAttribute(tag));
        string internalEntityId = PAIUtils::getInternalId(entityId);
        XMLString::transcode(TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* entityType = XMLString::transcode(entityElement->getAttribute(tag));
        XMLString::transcode(OWNER_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* entityOwnerId = XMLString::transcode(entityElement->getAttribute(tag));
        string internalEntityOwnerId = PAIUtils::getInternalId(entityOwnerId);
        XMLString::transcode(OWNER_NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* entityOwnerName = XMLString::transcode(entityElement->getAttribute(tag));
        logger().debug("PAI - Got Entity: id = %s (%s), type = %s, ownerId = %s (%s), ownerName = %s\n", entityId, internalEntityId.c_str(), entityType, entityOwnerId, internalEntityOwnerId.c_str(), entityOwnerName);

        resultHandle = AtomSpaceUtil::addNode(atomSpace, getSLObjectNodeType(entityType), internalEntityId.c_str());

        XMLString::release(&entityId);
        XMLString::release(&entityType);
        XMLString::release(&entityOwnerId);
        XMLString::release(&entityOwnerName);
        break;
    }
    default: {
        // Should never get here, but...
        throw opencog::RuntimeException(TRACE_INFO,
                                        "PAI - Undefined map from '%s' type to an Atom type.", paramType);
    }
    }

    XMLString::release(&paramName);
    XMLString::release(&paramType);
    XMLString::release(&paramValue);
    return resultHandle;
}

Handle PAI::processAgentType(const string& agentTypeStr, const string& internalAgentId)
{
    Handle agentNode;
    if ( agentTypeStr == AVATAR_OBJECT_TYPE ) {
        agentNode = AtomSpaceUtil::addNode(atomSpace, AVATAR_NODE, internalAgentId.c_str());
        logger().debug("PAI - agent %s is an avatar.\n", internalAgentId.c_str() );
    } else if ( agentTypeStr == "" || agentTypeStr == PET_OBJECT_TYPE ) {
        agentNode = AtomSpaceUtil::addNode(atomSpace, PET_NODE, internalAgentId.c_str());
        logger().debug("PAI - agent %s is a pet.\n", internalAgentId.c_str() );
    } else if ( agentTypeStr == HUMANOID_OBJECT_TYPE ) {
        agentNode = AtomSpaceUtil::addNode(atomSpace, HUMANOID_NODE, internalAgentId.c_str());
        logger().debug("PAI - agent %s is an humanoid.\n", internalAgentId.c_str() );
    } else {
        throw opencog::InvalidParamException(TRACE_INFO,
                     "PAI - Invalid value for agent type: '%s'", agentTypeStr.c_str( ) );
    } // else
    return agentNode;
}

void PAI::processAgentActionWithParameters(Handle& agentNode, const string& internalAgentId, unsigned long tsValue, const string& nameStr,  DOMElement* signal)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];
    // Add the perceptions into AtomSpace
    Handle predicateNode = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, ACTION_DONE_PREDICATE_NAME, true);
    Handle actionNode = AtomSpaceUtil::addNode(atomSpace, NODE, nameStr.c_str());

    HandleSeq parametersListLink;

    HandleSeq predicateListLinkOutgoing;
    predicateListLinkOutgoing.push_back(agentNode);
    predicateListLinkOutgoing.push_back(actionNode);

    XMLString::transcode(PARAMETER_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    DOMNodeList * list = signal->getElementsByTagName(tag);

    //used only in case the name of the action is 'grab'
    string grabObjectId;

    for (unsigned int i = 0; i < list->getLength(); i++) {
        DOMElement* paramElement = (DOMElement*) list->item(i);
        Handle ph = processAgentActionParameter(paramElement);
        parametersListLink.push_back(ph);

        //if action name is 'grab' then temporarly store
        //the argument to create isHolding predicate (see below)
        if (nameStr == "grab") {
            // Get the internalEntityId if the parameter was a grab
            // action
            grabObjectId = atomSpace.getName(ph);
        }
    } // for each parameter

    if ( parametersListLink.size( ) > 0 ) {
        predicateListLinkOutgoing.push_back( AtomSpaceUtil::addLink(atomSpace, LIST_LINK, parametersListLink ) );
    } // if

    Handle predicateListLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, predicateListLinkOutgoing);

    HandleSeq evalLinkOutgoing;
    evalLinkOutgoing.push_back(predicateNode);
    evalLinkOutgoing.push_back(predicateListLink);
    Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);
    Handle atTimeLink = atomSpace.getTimeServer().addTimeInfo(evalLink, tsValue);
    // JaredW: not sure if this is quite right, but it should have
    // some truth value (I think).
    // Probability of 1, count 1, of this action happening at that time.
    atomSpace.setTV(atTimeLink, SimpleTruthValue(1,1));
    AtomSpaceUtil::updateLatestAgentActionDone(atomSpace, atTimeLink, agentNode);

    // if the action is grab or drop created/update isHoldingSomething
    // and isHolding predicates
    if (nameStr == "grab") {
        AtomSpaceUtil::setupHoldingObject(atomSpace,
                                          internalAgentId,
                                          grabObjectId,
                                          getLatestSimWorldTimestamp());
    } else if (nameStr == "drop") {
        AtomSpaceUtil::setupHoldingObject(atomSpace,
                                          internalAgentId,
                                          string(""),
                                          getLatestSimWorldTimestamp());
    }

}

void PAI::processAgentActionPlanResult(char* agentID, unsigned long tsValue, const string& name, char* planIdStr, DOMElement* signal)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];
    string internalAgentId = PAIUtils::getInternalId(agentID);
    // getting status attribute value
    // NOTE: it's always present since it has a default value = "done"
    XMLString::transcode(STATUS_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* status = XMLString::transcode(signal->getAttribute(tag));

//std::cout<<"timestamp = "<<timestamp<<" status = "<<status<<" action-plan-id = "<<planIdStr<<" name = "<<name<<std::endl; 

    ActionStatus statusCode = opencog::pai::NONE;
    if (planIdStr && strlen(planIdStr)) {
        if (!strcmp(status, DONE_ACTION_STATUS)) {
            statusCode = opencog::pai::DONE;
//std::cout<<"ActionStatus: Done"<<std::endl; 
        } else if (!strcmp(status, ERROR_ACTION_STATUS)) {
            statusCode = opencog::pai::ERROR;
//std::cout<<"ActionStatus: ERROR"<<std::endl; 
        }
    }

    logger().warn("PAI - pet id %s (%s), name: %s, status: %s, statusCode: %d", agentID, internalAgentId.c_str(), name.c_str(), status, statusCode);

    // This is a feedback for a sent action plan
    XMLString::transcode(SEQUENCE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* sequenceStr = XMLString::transcode(signal->getAttribute(tag));

    if (planIdStr && strlen(planIdStr) > 0) {
        ActionPlanID planId = planIdStr;

        unsigned int sequence;
        if (sequenceStr && strlen(sequenceStr) > 0) {
            sequence = atoi(sequenceStr);
        } else {
            // all actions
            sequence = 0;
        }

        setActionPlanStatus(planId, sequence, statusCode, tsValue);
    } else {
        logger().error(
                     "PAI - Got a agent-signal with action '%s' with status (name" 
            " = '%s'), but no plan-id attribute!", name.c_str(), status);
    }
    XMLString::release(&sequenceStr);
    XMLString::release(&status);
}

unsigned long PAI::getLatestSimWorldTimestamp()
{
    return latestSimWorldTimestamp;
}

bool PAI::setLatestSimWorldTimestamp(unsigned long timestamp)
{
    if (timestamp > latestSimWorldTimestamp) {
        logger().debug("PAI - setLatestSimWorldTimestamp(%lu).", timestamp, latestSimWorldTimestamp);
        latestSimWorldTimestamp = timestamp;
    } else if (timestamp < latestSimWorldTimestamp) {
        logger().error("PAI - setLatestSimWorldTimestamp(%lu): Got a timestamp smaller than the latest received timestamp (%lu)!", timestamp, latestSimWorldTimestamp);
        return false;
    }
    return true;
}

unsigned long PAI::getTimestampFromElement(DOMElement* element) const
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];
    // getting timestamp attribute value
    XMLString::transcode(TIMESTAMP_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* timestamp = XMLString::transcode(element->getAttribute(tag));
    unsigned long result = getTimestampFromXsdDateTimeStr(timestamp);
    XMLString::release(&timestamp);
    return result;
}

unsigned long PAI::getTimestampFromXsdDateTimeStr(const char* xsdDateTimeStr) throw (opencog::RuntimeException, std::bad_exception)
{
// The number of possible matchs for the DateTime RegEx. This will change if the
// expression is altered.
#define REGEX_OUTPUT_SIZE 8


    // Date time format (complete)
    // YYYY-MM-DDThh:mm:ss.sTZD
    // where:
    //
    //  YYYY = four-digit year
    //  MM   = two-digit month (01=January, etc.)
    //  DD   = two-digit day of month (01 through 31)
    //  hh   = two digits of hour (00 through 23) (am/pm NOT allowed)
    //  mm   = two digits of minute (00 through 59)
    //  ss   = two digits of second (00 through 59)
    //  s    = one or more digits representing a decimal fraction of a second
    //  TZD  = time zone designator (Z or +hh:mm or -hh:mm)
    //

    logger().fine("PAI - getTimestampFromXsdDateTimeStr(%s)",
                 xsdDateTimeStr);

    boost::cmatch matches;
    boost::regex regularExpression;

    try {

        // Assignment and construction initialize the FSM used
        // for regexp parsing
        regularExpression.assign("(\\d{1,4}\\-\\d{1,2}\\-\\d{1,2})T(\\d{1,2}\\:\\d{1,2}\\:\\d{1,2})(\\.{1}\\d{1,9})?((Z)|(\\+{1}|\\-{1})(\\d{1,2}\\:\\d{1,2}))?");
    }

    catch (boost::regex_error& e) {
        throw opencog::RuntimeException(TRACE_INFO, "PAI - Invalid regular expression (%s).\n %s",
                                        e.what(), regularExpression.str().c_str());
    }


    if (!boost::regex_match(xsdDateTimeStr, matches, regularExpression)) {
        throw opencog::RuntimeException(TRACE_INFO,
                                        "PAI - DateTime String '%s' does not match regular expression.",
                                        xsdDateTimeStr);
    }

    if (matches.size() != REGEX_OUTPUT_SIZE) {
        logger().warn(
                     "PAI - RegEx result size (%d) differs from predifined one (%d).",
                     matches.size(), REGEX_OUTPUT_SIZE);
        // return what???
    }

    // TODO: Change to reference date to one that everyone agrees (or that makes more sense)
    // IMPORTANT: date already in UTC time

    const ptime EPOCH = PAIUtils::getSystemEpoch( );

    // getting date - MUST be provided
    std::string dateStr(matches[1].first, matches[1].second);
    //std::cout << "Date "  << dateStr << std::endl;

    // getting time date - MUST be provided
    std::string timeStr(matches[2].first, matches[2].second);
    //std::cout << "Time " << timeStr << std::endl;

    // ckecking if time has miliseconds
    std::string milisec(matches[3].first, matches[3].second);
    //std::cout << "Milisec " <<  milisec << std::endl;
    if (!milisec.empty()) {
        timeStr.append(milisec);
    } else {
        timeStr.append(std::string(".000"));
    }

    // time received
    time_duration timeReceived(duration_from_string(timeStr));
    date dateReceived(from_string(dateStr));

    ptime dateTimeReceived(dateReceived, timeReceived);

    std::string utcTime(matches[5].first, matches[5].second);
    //std::cout << "UTC presence: " << utcTime << std::endl;
    if (utcTime.empty()) {

        // need to put time_duration in the UTC time. Getting timezone offset
        // info
        std::string timezoneOffset(matches[7].first, matches[7].second);
        timezoneOffset.append(std::string(":00.000"));
        //std::cout << "Timezone" << timezoneOffset << std::endl;

        // getting timezone signal
        //
        // IMPORTANT: if '+' the offset should be subtracted from time. if '-'
        // the offset should be added to time.
        std::string timezone(matches[6].first, matches[6].second);
        if (timezone == "+") {
            dateTimeReceived = dateTimeReceived - duration_from_string(timezoneOffset);
        } else if (timezone == "-") {
            dateTimeReceived = dateTimeReceived + duration_from_string(timezoneOffset);
        }
    }

    time_duration td = dateTimeReceived - EPOCH;
    unsigned long elapsedSeconds = (unsigned long) td.total_seconds();

    time_duration td_seconds = seconds(elapsedSeconds);
    unsigned long milliseconds  = (unsigned long)((td - td_seconds).total_milliseconds());

#ifdef DATETIME_DECIMAL_RESOLUTION
    unsigned long result = elapsedSeconds * 10 + milliseconds / 100;
#else
    unsigned long result = elapsedSeconds * 100 + milliseconds / 10;
#endif

    //logger().debug("PAI - ulong timestamp = %lu, converted from %s", result, xsdDateTimeStr);
    return result;
}

void PAI::processInstruction(DOMElement * element)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    unsigned long tsValue = getTimestampFromElement(element);

    // Note: The time stamp from RelexServer represents whatever timestamp
    // it was originally sent by this OAC, so the result of this call is
    // safe to ignore.
    setLatestSimWorldTimestamp(tsValue);
    
    // get id of destination agent for instruction
    XMLString::transcode(INSTRUCTION_TO_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* petID = XMLString::transcode(element->getAttribute(tag));
    string internalPetId = PAIUtils::getInternalId(petID);
    // TODO: Do we need to check if petID matches the id of the Pet being
    // controlled by this OAC?

    // get source agent id
    XMLString::transcode(INSTRUCTION_FROM_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* avatarID = XMLString::transcode(element->getAttribute(tag));
    string internalAvatarId = PAIUtils::getInternalId(avatarID);

    // get content-type attribute value
    XMLString::transcode(CONTENT_TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* contentType = XMLString::transcode(element->getAttribute(tag));

    // get target-mode attribute value
    XMLString::transcode(TARGET_MODE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* targetMode = XMLString::transcode(element->getAttribute(tag));
    
    OC_ASSERT(strlen(contentType) > 0 );
    OC_ASSERT(strlen(targetMode) > 0 );

    // get the value of the sentence
    XMLString::transcode(SENTENCE_TYPE, tag, PAIUtils::MAX_TAG_LENGTH);
    DOMNodeList *list = element->getElementsByTagName(tag);
    OC_ASSERT(list->getLength() == 1);
    DOMElement* sentenceElement = (DOMElement*) list->item(0);
    char* sentenceText = XMLString::transcode(sentenceElement->getTextContent());
    // get the sentence length
    XMLString::transcode(LENGTH_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* length = XMLString::transcode(sentenceElement->getAttribute(tag));    
    int sentenceLength = atoi( length );
    XMLString::release(&length);
    OC_ASSERT(static_cast<int>( strlen( sentenceText) ) == sentenceLength );
    XMLString::trim(sentenceText);

    // get the value of the parsed-sentence
    //XMLString::transcode(PARSED_SENTENCE_TYPE, tag, PAIUtils::MAX_TAG_LENGTH);
    XMLString::transcode(PARSED_SENTENCE_TYPE, tag, 56000);
    list = element->getElementsByTagName(tag);
    OC_ASSERT(list->getLength( ) == 0 || list->getLength( ) == 1 );

    char* parsedSentenceText = NULL;
    int parsedSentenceLength = 0;
    if ( list->getLength() == 1 ) {
        DOMElement* parsedSentenceElement = (DOMElement*) list->item(0);
        parsedSentenceText = XMLString::transcode(parsedSentenceElement->getTextContent());
        /// getting the sentence length
        XMLString::transcode(LENGTH_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        length = XMLString::transcode(parsedSentenceElement->getAttribute(tag));
        parsedSentenceLength = atoi( length );
        XMLString::release(&length);
        OC_ASSERT(static_cast<int>( strlen( parsedSentenceText ) ) == parsedSentenceLength );
        XMLString::trim(parsedSentenceText);
    } // if

#ifdef HAVE_GUILE
    if(parsedSentenceText != NULL){
        SchemeEval & evaluator = SchemeEval::instance(); 
        std::string scheme_expression, scheme_return_value; 

        // Clean up all the information of previous sentence, such as detach 
        // the AnchorNode of previous sentence if it has not been processed. 
        scheme_expression = "( reset_dialog_system )";
        scheme_return_value = evaluator.eval(scheme_expression); 
        if ( evaluator.eval_error() ) {
            logger().error("PAI::%s - Failed to execute '%s'", 
                            __FUNCTION__, 
                           scheme_expression.c_str()
                          ); 
        }

        // Put the newly parsed sentence into AtomSpace, then PsiActionSelectionAgent 
        // and dialog_system.scm will continue processing it. 
        scheme_return_value = evaluator.eval(parsedSentenceText); 
        if ( evaluator.eval_error() ) {
            logger().error("PAI::%s - Failed to put the parsed result from RelexServer to AtomSpace", 
                           __FUNCTION__
                          ); 
        }
    }    
#endif

    // TODO: some code below may be not necessary, since the PsiActionSelectionAgent
    //       and dialog_system.scm will do most of the work instead. 

    // Add the perceptions into AtomSpace

    Handle predicateNode = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, ACTION_DONE_PREDICATE_NAME, true);
    Handle saySchemaNode = AtomSpaceUtil::addNode(atomSpace, GROUNDED_SCHEMA_NODE, SAY_SCHEMA_NAME, true);
    Handle agentNode = AtomSpaceUtil::getAgentHandle(atomSpace, internalAvatarId);
    if (agentNode == Handle::UNDEFINED) {
        agentNode = AtomSpaceUtil::addNode(atomSpace, AVATAR_NODE, internalAvatarId);
    } 

    string sentence = "to:";
    sentence += internalPetId;
    sentence += ": ";
    sentence += sentenceText;
    //logger().debug("sentence = '%s'\n", sentence.c_str());

    Handle sentenceNode = AtomSpaceUtil::addNode(atomSpace, SENTENCE_NODE, sentence.c_str());

    if ( avatarInterface.getPetId( ) == internalPetId ) {
        AtomSpaceUtil::setPredicateValue( atomSpace, "heard_sentence",
            TruthValue::TRUE_TV( ), sentenceNode );
    } // if


    HandleSeq schemaListLinkOutgoing;
    schemaListLinkOutgoing.push_back(agentNode);
    schemaListLinkOutgoing.push_back(sentenceNode);
    Handle schemaListLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, schemaListLinkOutgoing);

    if ( parsedSentenceText != NULL ) {
        // ok there is an incoming parsed (by relex sentece)
        // so, connect the author of the Relex sentences to each sentence
        // it will be useful to identify which agent says each sentence
        HandleSeq sentenceOwner(2);
        sentenceOwner[0] = agentNode;

        logger().debug( "PAI::%s - Connecting sentences to its respective owners",
                        __FUNCTION__ );
                        
        Handle anchorNode = atomSpace.getHandle( ANCHOR_NODE, "# New Parsed Sentence");
        HandleSeq incomingSet = atomSpace.getIncoming(anchorNode);
        unsigned int i;
        bool linkFound = false;
        for( i = 0; !linkFound && i < incomingSet.size( ); ++i ) {
            if ( atomSpace.getType( incomingSet[i] ) == LIST_LINK ) {
                logger().debug( "PAI::%s - LIST_LINK found, now inspect it to identify the sentences",
                                __FUNCTION__);
                HandleSeq outgoingSet = atomSpace.getOutgoing( incomingSet[i] );
                if ( outgoingSet.size( ) == 0 || outgoingSet[0] != anchorNode ) {
                    logger().debug( "PAI::%s - Wrong LIST_LINK arity[%d]",
                                    __FUNCTION__, outgoingSet.size() );
                    continue;
                } // if

                linkFound = true;
                unsigned int j;
                for( j = 1; j < outgoingSet.size( ); ++j ) {
                    if ( atomSpace.getType( outgoingSet[j] ) == SENTENCE_NODE ) {
                        sentenceOwner[1] = outgoingSet[j];
                        Handle sentenceOwnerLink = 
                            AtomSpaceUtil::addLink(atomSpace, LIST_LINK, sentenceOwner );
                        atomSpace.setTV( sentenceOwnerLink, TruthValue::TRUE_TV( ) );
                        logger().debug( "PAI::%s - Sentence found. now connecting[%s]",
                                        __FUNCTION__, atomSpace.getName( sentenceOwner[1] ).c_str( ) );

                    } // if                    
                } // for
            } // if
        } // for

    } // if


    HandleSeq execLinkOutgoing;
    execLinkOutgoing.push_back(saySchemaNode);
    execLinkOutgoing.push_back(schemaListLink);
    Handle execLink = AtomSpaceUtil::addLink(atomSpace, EXECUTION_LINK, execLinkOutgoing);

    HandleSeq predicateListLinkOutgoing;
    predicateListLinkOutgoing.push_back(execLink);
    Handle predicateListLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, predicateListLinkOutgoing);

    HandleSeq evalLinkOutgoing;
    evalLinkOutgoing.push_back(predicateNode);
    evalLinkOutgoing.push_back(predicateListLink);
    Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);
    Handle atTimeLink = atomSpace.getTimeServer().addTimeInfo(evalLink, tsValue);
    AtomSpaceUtil::updateLatestAvatarSayActionDone(atomSpace, atTimeLink, agentNode);


    std::vector<std::string> arguments;
    arguments.push_back( sentenceText );

    if ( std::string( contentType ) == "FACT" ) {

        avatarInterface.getCurrentModeHandler( ).handleCommand( "storeFact", arguments );

    } else if ( std::string( contentType ) == "COMMAND" ) {
        avatarInterface.getCurrentModeHandler( ).handleCommand( "evaluateSentence", arguments );

    } else if ( std::string( contentType ) == "QUESTION" ) {
        AtomSpaceUtil::setPredicateValue( atomSpace,
            "is_question", TruthValue::TRUE_TV( ), sentenceNode );
        AtomSpaceUtil::setPredicateValue( atomSpace,
            "was_answered", TruthValue::FALSE_TV( ), sentenceNode );
        
        avatarInterface.getCurrentModeHandler( ).handleCommand( "answerQuestion", arguments );

    } else if ( std::string( contentType ) == "SPECIFIC_COMMAND" ) {
        if ( std::string( targetMode ) == avatarInterface.getCurrentModeHandler( ).getModeName( ) ) {

            // ATTENTION: a sentence must be upper case to be handled by the agent mode handlers
            boost::to_upper(arguments[0]);            
            arguments.push_back( boost::lexical_cast<std::string>( tsValue ) );
            arguments.push_back( internalAvatarId );
            avatarInterface.getCurrentModeHandler( ).handleCommand( "instruction", arguments );
        } else {
            logger().debug( "PAI::%s - A specific command of another mode was sent. Ignoring it. Current Mode: %s, Target Mode: %s, Command: %s", 
                            __FUNCTION__, avatarInterface.getCurrentModeHandler( ).getModeName( ).c_str( ), targetMode, sentenceText );
        } // else
    } // if


    XMLString::release(&petID);
    XMLString::release(&avatarID);
    XMLString::release(&contentType);
    XMLString::release(&targetMode);
    XMLString::release(&sentenceText);
    XMLString::release(&parsedSentenceText);
}

void PAI::processAgentSensorInfo(DOMElement * element)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    // Gets each blip element
    XMLString::transcode(PERCEPTION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    DOMNodeList *perceptionList = element->getElementsByTagName(tag);

    logger().debug("PAI - Received an agent-sensor-info. %d perceptions", perceptionList->getLength() );

    for (unsigned int i = 0; i < perceptionList->getLength(); i++) {
        DOMElement* perceptionElement = (DOMElement*) perceptionList->item(i);

        XMLString::transcode(SENSOR_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* sensor = XMLString::transcode(perceptionElement->getAttribute(tag));

        XMLString::transcode(SUBJECT_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* subject = XMLString::transcode(perceptionElement->getAttribute(tag));

        XMLString::transcode(SIGNAL_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* signal = XMLString::transcode(perceptionElement->getAttribute(tag));

        logger().debug("PAI - Received a perception: %s, %s, %s", sensor, subject, signal );


        if ( !strcmp( sensor, "visibility" ) && !strcmp( subject, "map" ) ) {
            std::vector<std::string> arguments;
            arguments.push_back( signal );
            avatarInterface.getCurrentModeHandler( ).handleCommand( "visibilityMap", arguments );
        } // if

        XMLString::release(&sensor);
        XMLString::release(&subject);
        XMLString::release(&signal);

    } // for
}



void PAI::processTempSignal(DOMElement * element) throw (opencog::RuntimeException, opencog::InvalidParamException, std::bad_exception)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    // getting timestamp attribute value
    unsigned long tsValue = getTimestampFromElement(element);
    if (!setLatestSimWorldTimestamp(tsValue)) {
        logger().error("PAI - Received old timestamp in temp-signal => Message discarded!");
        return;
    }

    // getting avatar id attribute value
    XMLString::transcode(AVATAR_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* avatarID = XMLString::transcode(element->getAttribute(tag));
    string internalAvatarId = PAIUtils::getInternalId(avatarID);

    DOMNodeList * signals = element->getChildNodes();
    for (unsigned int i = 0; i < signals->getLength(); i++) {
        DOMNode* n = signals->item(i);
        if (n->getNodeType() != DOMNode::ELEMENT_NODE)
            continue;
        DOMElement* signal = (DOMElement*) n;
        
        // getting action name attribute value
        XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* name = XMLString::transcode(signal->getAttribute(tag));
        string nameStr(camelCaseToUnderscore(name));

        char* signalName = XMLString::transcode(signal->getTagName());
        // We only know how to deal with action elements
        if (strcmp(signalName,ACTION_ELEMENT) != 0) continue;

        logger().debug("PAI - Got temp-signal: avatarId = %s (%s), name = %s, timestamp = %u\n", avatarID, internalAvatarId.c_str(), name, tsValue);

        // Add the perceptions into AtomSpace
        Handle predicateNode = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, ACTION_DONE_PREDICATE_NAME, true);
        Handle avatarNode = AtomSpaceUtil::addNode(atomSpace, AVATAR_NODE, internalAvatarId.c_str());
        Handle actionNode = AtomSpaceUtil::addNode(atomSpace, NODE, nameStr.c_str());

        HandleSeq predicateListLinkOutgoing;
        predicateListLinkOutgoing.push_back(avatarNode);
        predicateListLinkOutgoing.push_back(actionNode);

        XMLString::transcode(PARAMETER_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        DOMNodeList * list = element->getElementsByTagName(tag);

        //used only in case the name of the action is 'grab'
        string grabObjectId;

        for (unsigned int i = 0; i < list->getLength(); i++) {
            DOMElement* paramElement = (DOMElement*) list->item(i);

            XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
            char* paramName = XMLString::transcode(paramElement->getAttribute(tag));

            XMLString::transcode(TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
            char* paramType = XMLString::transcode(paramElement->getAttribute(tag));

            XMLString::transcode(VALUE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
            char* paramValue = XMLString::transcode(paramElement->getAttribute(tag));

            logger().debug("PAI - Got param: name = %s, type = %s, value = %s", paramName, paramType, paramValue);

            // This function can throw an InvalidParamException
            const ActionParamType& actionParamType = ActionParamType::getFromName(paramType);

            switch (actionParamType.getCode()) {
            case BOOLEAN_CODE: {
                if (strcmp(paramValue, "true") && strcmp(paramValue, "false")) {
                    throw opencog::InvalidParamException(TRACE_INFO,
                                                         "PAI - Invalid value for boolean parameter: '%s'. It should be true or false.", paramValue);
                }
                predicateListLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, paramValue));
                break;
            }
            case INT_CODE:
            case FLOAT_CODE: {
                // TODO: check if string is a valid number?
                predicateListLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, paramValue));
                break;
            }
            case STRING_CODE: {
                // TODO: converts the string to lowercase?
                predicateListLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, paramValue));
                break;
            }
            case VECTOR_CODE: {
                XMLString::transcode(VECTOR_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
                DOMNodeList * vectorList = paramElement->getElementsByTagName(tag);
                if (vectorList->getLength() <= 0) {
                    throw opencog::InvalidParamException(TRACE_INFO,
                                                         "PAI - Got a parameter element with vector type without a vector child element.");
                }
                DOMElement* vectorElement = (DOMElement*) vectorList->item(0);
                XMLString::transcode(X_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* xStr = XMLString::transcode(vectorElement->getAttribute(tag));
                XMLString::transcode(Y_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* yStr = XMLString::transcode(vectorElement->getAttribute(tag));
                XMLString::transcode(Z_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* zStr = XMLString::transcode(vectorElement->getAttribute(tag));
                logger().debug("PAI - Got vector: x = %s, y = %s, z = %s\n", xStr, yStr, zStr);
                HandleSeq rotationListLink;
                // TODO: check if string is a valid number?
                rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, xStr));
                rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, yStr));
                rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, zStr));
                predicateListLinkOutgoing.push_back(AtomSpaceUtil::addLink(atomSpace, LIST_LINK, rotationListLink));

                XMLString::release(&xStr);
                XMLString::release(&yStr);
                XMLString::release(&zStr);
                break;
            }
            case ROTATION_CODE: {
                XMLString::transcode(ROTATION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
                DOMNodeList * rotationList = paramElement->getElementsByTagName(tag);
                if (rotationList->getLength() <= 0) {
                    throw opencog::InvalidParamException(TRACE_INFO,
                                                         "PAI - Got a parameter element with Rotation type without a rotation child element.");
                }
                DOMElement* rotationElement = (DOMElement*) rotationList->item(0);
                XMLString::transcode(PITCH_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* pitchStr = XMLString::transcode(rotationElement->getAttribute(tag));
                XMLString::transcode(ROLL_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* rollStr = XMLString::transcode(rotationElement->getAttribute(tag));
                XMLString::transcode(YAW_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* yawStr = XMLString::transcode(rotationElement->getAttribute(tag));
                logger().debug("PAI - Got rotaion: pitch = %s, roll = %s, yaw = %s\n", pitchStr, rollStr, yawStr);

                HandleSeq rotationListLink;
                // TODO: check if string is a valid number?
                rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, pitchStr));
                rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, rollStr));
                rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, yawStr));
                predicateListLinkOutgoing.push_back(AtomSpaceUtil::addLink(atomSpace, LIST_LINK, rotationListLink));

                XMLString::release(&pitchStr);
                XMLString::release(&rollStr);
                XMLString::release(&yawStr);
                break;
            }
            case ENTITY_CODE: {
                XMLString::transcode(ENTITY_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
                DOMNodeList * entityList = paramElement->getElementsByTagName(tag);
                if (entityList->getLength() <= 0) {
                    throw opencog::InvalidParamException(TRACE_INFO,
                                                         "PAI - Got a parameter element with Entity type without an entity child element.");
                }
                DOMElement* entityElement = (DOMElement*) entityList->item(0);

                XMLString::transcode(ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* entityId = XMLString::transcode(entityElement->getAttribute(tag));
                string internalEntityId = PAIUtils::getInternalId(entityId);
                XMLString::transcode(TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* entityType = XMLString::transcode(entityElement->getAttribute(tag));
                XMLString::transcode(OWNER_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* entityOwnerId = XMLString::transcode(entityElement->getAttribute(tag));
                string internalEntityOwnerId = PAIUtils::getInternalId(entityOwnerId);
                XMLString::transcode(OWNER_NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* entityOwnerName = XMLString::transcode(entityElement->getAttribute(tag));
                logger().debug("PAI - Got Entity: id = %s (%s), type = %s, ownerId = %s (%s), ownerName = %s\n", entityId, internalEntityId.c_str(), entityType, entityOwnerId, internalEntityOwnerId.c_str(), entityOwnerName);
                //if action name is 'grab' then temporarly store
                //the argument to create isHolding predicate (see below)
                if (nameStr == "grab") {
                    grabObjectId = internalEntityId;
                }

                predicateListLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, getSLObjectNodeType(entityType), internalEntityId.c_str()));

                XMLString::release(&entityId);
                XMLString::release(&entityType);
                XMLString::release(&entityOwnerId);
                XMLString::release(&entityOwnerName);
                break;
            }
            default: {
                // Should never get here, but...
                throw opencog::RuntimeException(TRACE_INFO,
                                                "PAI - Undefined map from '%s' type to an Atom type.", paramType);
            }
            }

            XMLString::release(&paramName);
            XMLString::release(&paramType);
            XMLString::release(&paramValue);
        } // for each parameter

        Handle predicateListLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, predicateListLinkOutgoing);

        HandleSeq evalLinkOutgoing;
        evalLinkOutgoing.push_back(predicateNode);
        evalLinkOutgoing.push_back(predicateListLink);
        Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);
        Handle atTimeLink = atomSpace.getTimeServer().addTimeInfo(evalLink, tsValue);
        AtomSpaceUtil::updateLatestAvatarActionDone(atomSpace, atTimeLink, avatarNode);

        //if the action is grab or drop created/update isHoldingSomething
        //and isHolding predicates
        if (nameStr == "grab") {
            AtomSpaceUtil::setupHoldingObject(atomSpace,
                                              internalAvatarId,
                                              grabObjectId,
                                              getLatestSimWorldTimestamp());
        } else if (nameStr == "drop") {
            AtomSpaceUtil::setupHoldingObject(atomSpace,
                                              internalAvatarId,
                                              string(""),
                                              getLatestSimWorldTimestamp());
        }
        XMLString::release(&name);
    }

    XMLString::release(&avatarID);
}

Type PAI::getSLObjectNodeType(const char* objectType)
{
    Type result;
    if (strcmp(objectType, AVATAR_OBJECT_TYPE) == 0) {
        result = AVATAR_NODE;
    } else if (strcmp(objectType, PET_OBJECT_TYPE) == 0 ) {
        result = PET_NODE;
    } else if (strcmp(objectType, HUMANOID_OBJECT_TYPE) == 0) {
        result = HUMANOID_NODE;
    } else if (strcmp(objectType, STRUCTURE_OBJECT_TYPE) == 0) {
        result = STRUCTURE_NODE;
    } else if (strcmp(objectType, ACCESSORY_OBJECT_TYPE) == 0) {
        result = ACCESSORY_NODE;
    } else {
        result = OBJECT_NODE;
    }
    return result;
}

double PAI::getPositionAttribute(DOMElement * element, const char* tagName)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];
    XMLString::transcode(tagName, tag, PAIUtils::MAX_TAG_LENGTH);
    char* strvalue = XMLString::transcode(element->getAttribute(tag));
    if (!strlen(strvalue)) {
        logger().error("PAI - processMapInfo(): got no %s attribute", tagName);
    }
    double result = atof(strvalue);
    XMLString::release(&strvalue);
    return result;
}

void PAI::processMapInfo(DOMElement * element, HandleSeq &toUpdateHandles)
{
    struct timeval timer_start, timer_end;
    time_t elapsed_time = 0;
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    logger().debug("PAI - processMapInfo(): init.");

    // gets global position offset (to be added to global x and y positions to get map dimensions)
    double offset = getPositionAttribute(element, GLOBAL_POS_OFFSET_ATTRIBUTE);
    logger().fine("PAI - processMapInfo(): global position offset = %s => offset = %lf", GLOBAL_POS_OFFSET_ATTRIBUTE, offset);

    // gets global position x
    xMin = getPositionAttribute(element, GLOBAL_POS_X_ATTRIBUTE);
    xMax = xMin + offset;
    logger().fine("PAI - processMapInfo(): global position x = %s => xMin = %lf, xMax = %lf", GLOBAL_POS_X_ATTRIBUTE, xMin, xMax);

    // gets global position y
    yMin = getPositionAttribute(element, GLOBAL_POS_Y_ATTRIBUTE);
    yMax = yMin + offset;
    logger().fine("PAI - processMapInfo(): global position y = %s => yMin = %lf, yMax = %lf", GLOBAL_POS_Y_ATTRIBUTE, yMin, yMax);

    // gets global level height
    floorHeight = getPositionAttribute(element, GLOBAL_FLOOR_HEIGHT_ATTRIBUTE);
    logger().debug("PAI - processMapInfo(): global floor height = %lf", floorHeight);


    // getting grid map dimensions from system parameters
    unsigned int xDim = opencog::config().get_int("MAP_XDIM");
    unsigned int yDim = opencog::config().get_int("MAP_YDIM");

    SpaceServer& spaceServer = atomSpace.getSpaceServer();
    spaceServer.setMapBoundaries(xMin, xMax, yMin, yMax, xDim, yDim, floorHeight);

    bool keepPreviousMap = avatarInterface.isExemplarInProgress();
    // Gets each blip element
    XMLString::transcode(BLIP_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    DOMNodeList * blipList = element->getElementsByTagName(tag);

    //printf("Blip List: %d\n",blipList->getLength());

    for (unsigned int i = 0; i < blipList->getLength(); i++) {
        gettimeofday(&timer_start, NULL);

        DOMElement* blipElement = (DOMElement*) blipList->item(i);

        unsigned long tsValue = getTimestampFromElement(blipElement);
        if (!setLatestSimWorldTimestamp(tsValue)) {
            logger().error("PAI - Received old timestamp in blip => Message discarded!");
            continue;
        }

        // Gets each properties element
        XMLString::transcode(PROPERTIES_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        DOMNodeList * propertiesList = blipElement->getElementsByTagName(tag);

        // printf("Properties List: %d\n",propertiesList->getLength());
        std::string detector="";
        bool objRemoval=false;
        bool isObjectInsidePetFov=false;
        double length=0.0;
        double width=0.0;
        double height=0.0;
        bool edible=false;
        bool drinkable=false;

        bool petHome=false;
        bool foodBowl=false;
        bool waterBowl=false;

        // Currently, we don't need to update predicates (e.g. isSmall, isMovable)
        // of terrain object. So when this flag is set to true, we don't need to add
        // this terrain object in the "toUpdateHandles" set, just insert it into space map.
        bool isTerrainObject = false;

        std::string entityClass="unknown";
        std::map<std::string, float> colors;
        std::string material="";
        std::string texture="";
        bool isToy=false;

        for (unsigned int j = 0; j < propertiesList->getLength(); j++) {

            DOMElement* propertiesElement = 
                     (DOMElement*) propertiesList->item(j);

            XMLString::transcode(PROPERTY_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
            DOMNodeList * propertyList = propertiesElement->getElementsByTagName(tag);

            //printf("Property List: %d\n",propertyList->getLength());
            
            for (unsigned int p = 0; p < propertyList->getLength(); p++) {

                DOMElement* propertyElement = 
                         (DOMElement*) propertyList->item(p);


                XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* name =  XMLString::transcode(propertyElement->getAttribute(tag));

                XMLString::transcode(VALUE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
                char* value =  XMLString::transcode(propertyElement->getAttribute(tag));

        
                // Note from Tristan: All objects (toys, pets, and pet-owners) report map updates, usually on themselves,
                // but sometimes on other avatars or objects (such as structures).
                // The detector boolean simply means that the blip indicated was the one that the object reported on itself.     
                if( strcmp(name, DETECTOR_ATTRIBUTE) == 0 ){
                    detector = value;
                } else if( strcmp(name, REMOVE_ATTRIBUTE) == 0 ){
                    //objRemoval = !strcmp(value, "true");
                    objRemoval = (strcmp(value, "true") == 0 ? true : false);
                } 
                // visibility-status
                else if( strcmp(name, VISIBILITY_STATUS_ATTRIBUTE) == 0 ){
                   isObjectInsidePetFov = !strcmp(value, "visible" );
                } 
                // length
                else if( strcmp(name, LENGTH_ATTRIBUTE) == 0 ){
                   length = atof(value);
                } 
                // width
                else if( strcmp(name, WIDTH_ATTRIBUTE) == 0 ){
                   width = atof(value);
                } 
                // height
                else if( strcmp(name, HEIGHT_ATTRIBUTE) == 0 ){
                   height = atof(value);
                } 
                // edible
                else if( strcmp(name, EDIBLE_ATTRIBUTE) == 0 ){
                    edible = (strcmp(value, "true") == 0 ? true : false);
                } 
                // drinkable
                else if( strcmp(name, DRINKABLE_ATTRIBUTE) == 0 ){
                    drinkable = (strcmp(value, "true") == 0 ? true : false);
                } 
                // pet home
                else if( strcmp(name, PET_HOME_ATTRIBUTE) == 0 ){
                    petHome = (strcmp(value, "true") == 0 ? true : false);
                } 
                // food bowl
                else if( strcmp(name, FOOD_BOWL_ATTRIBUTE) == 0 ){
                    foodBowl = (strcmp(value, "true") == 0 ? true : false);
                } 
                // water bowl
                else if( strcmp(name, WATER_BOWL_ATTRIBUTE) == 0 ){
                    waterBowl = (strcmp(value, "true") == 0 ? true : false);
                } 
                //entity class
                else if( strcmp(name, ENTITY_CLASS_ATTRIBUTE) == 0 ){
                    entityClass = value;

                    // Check if this entity is a terrain object(a.k.a a block)
                    if (entityClass == "block")
                        isTerrainObject = true;
                } 
                //color
                else if( strncmp(name, COLOR_ATTRIBUTE, strlen(COLOR_ATTRIBUTE)) == 0 ){
                    // color properties in the following format:
                    // color-<sequencial>
                    char *savePtr;
                    char *color = strtok_r(value, "-%", &savePtr);
                    char *percent = strtok_r(NULL, "-%", &savePtr);
                    if (percent != NULL) {
                        colors[color] = atof(percent)/100.0f;
                    } else {
                        logger().error("Got an invalid value for color attribute in blip element");
                    }
                } 
                // is toy
                else if( strcmp(name, IS_TOY_ATTRIBUTE) == 0 ){
                   isToy = (strcmp(value, "true") == 0 ? true : false);
                } 
                //material
                else if( strcmp(name, MATERIAL_ATTRIBUTE) == 0 ){
                    material = value;
                } 
                //texture
                else if( strcmp(name, TEXTURE_ATTRIBUTE) == 0 ){
                    texture = value;
                }


                XMLString::release(&value);
                XMLString::release(&name);

          }// for property
        } // for properties
        
        // processing entity element of blip
        XMLString::transcode(ENTITY_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
        DOMElement* entityElement =
            (DOMElement*) blipElement->getElementsByTagName(tag)->item(0);

        XMLString::transcode(ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* entityId = XMLString::transcode(entityElement->getAttribute(tag));
        string internalEntityId = PAIUtils::getInternalId(entityId);

        bool isPetObject = (internalEntityId == avatarInterface.getPetId());
        bool isPetOwner = (internalEntityId == avatarInterface.getOwnerId());
        // NOTE: blip for the pet itself should be the first one. This way, owner's id will be already known
        // at avatarInterface when blip for the onwer comes.

        Handle objectNode;
        XMLString::transcode(TYPE_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* entityType = XMLString::transcode(entityElement->getAttribute(tag));

        Handle typeNode = Handle::UNDEFINED;

//     logger().info("PAI - processMapInfo(): entityType=%s", entityType);
        if (entityType && strlen(entityType)) {
            objectNode = AtomSpaceUtil::addNode(atomSpace, getSLObjectNodeType(entityType), internalEntityId.c_str());
            typeNode = AtomSpaceUtil::addNode(atomSpace, NODE, entityType);
            HandleSeq inheritanceLinkOutgoing;
            inheritanceLinkOutgoing.push_back(objectNode);
            inheritanceLinkOutgoing.push_back(typeNode);

            AtomSpaceUtil::addLink(atomSpace, INHERITANCE_LINK, inheritanceLinkOutgoing);

        } else {
            objectNode = AtomSpaceUtil::addNode(atomSpace, OBJECT_NODE, internalEntityId.c_str());
        }
        XMLString::transcode(NAME_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* objName = XMLString::transcode(entityElement->getAttribute(tag));

//     logger().info("PAI - processMapInfo(): objName=%s", objName);

        if (objName && strlen(objName)) {
            if (isPetObject)  {
                avatarInterface.setName(objName);
            }

            Handle objNameNode = AtomSpaceUtil::addNode(atomSpace, WORD_NODE, objName, isPetObject || isPetOwner);
            HandleSeq outgoing;
            outgoing.push_back(objNameNode);
            outgoing.push_back(objectNode);
            AtomSpaceUtil::addLink(atomSpace, WR_LINK, outgoing, true);
        }

        XMLString::transcode(OWNER_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
        char* ownerId = XMLString::transcode(entityElement->getAttribute(tag));

        if (ownerId && strlen(ownerId)) {
            string internalOwnerId = PAIUtils::getInternalId(ownerId);
//            logger().log(Util::Logger::INFO, "PAI - processMapInfo(): ownerId=%s (%s)", ownerId, internalOwnerId.c_str());
            if (isPetObject) {
                avatarInterface.setOwnerId(internalOwnerId);
            }

            // TODO: we will assume that only Avatars can own something or someone
            // (pets). This can be changed if necessary.
            // Carlos Lopes - 24/10/2007
            Handle ownerNode = AtomSpaceUtil::addNode(atomSpace, AVATAR_NODE, internalOwnerId.c_str(), isPetObject);
            addOwnershipRelation(ownerNode, objectNode, isPetObject);
        }


        if (!isTerrainObject) {
            // objectNode with new/updated atom created add to ToUpdateHandles
            toUpdateHandles.push_back(objectNode);
        }

        // Update existance predicate 
        addPropertyPredicate(std::string("exist"), objectNode, !objRemoval, false);

        if (objRemoval) {
            atomSpace.getSpaceServer().removeSpaceInfo(keepPreviousMap, objectNode, tsValue);

            // check if the object to be removed is marked as grabbed in
            // AvatarInterface. If so grabbed status should be unset.
            if (avatarInterface.hasGrabbedObj() &&
                    avatarInterface.getGrabbedObj() == atomSpace.getName(objectNode)) {
                avatarInterface.setGrabbedObj(std::string(""));
            }
        } else {
            // position
            XMLString::transcode(POSITION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
            DOMElement* positionElement =
                (DOMElement*) blipElement->getElementsByTagName(tag)->item(0);

            // rotation
            XMLString::transcode(ROTATION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
            DOMElement* rotationElement =
                (DOMElement*) blipElement->getElementsByTagName(tag)->item(0);

            // velocity
            XMLString::transcode(VELOCITY_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
            DOMElement* velocityElement =
                (DOMElement*) blipElement->getElementsByTagName(tag)->item(0);
            Vector velocityVector = getVelocityData(velocityElement);

            // check the velocity vector in order to see if the pet is moving,
            // i.e., vector mag greater than zero
            bool moving = false;
            if (velocityVector.x != 0.0 ||
                    velocityVector.y != 0.0 ||
                    velocityVector.z != 0.0) {
                moving = true;
            }

//      logger().log(Util::Logger::INFO, "PAI - processMapInfo(): before addSpacePredicates");

            // TODO: velocity vector is not being added to SpaceMaps. Add it when needed.
            addSpacePredicates(keepPreviousMap, objectNode, tsValue,
                               positionElement, rotationElement,
                               length, width, height, edible, drinkable, isTerrainObject);

            gettimeofday(&timer_end, NULL);
            elapsed_time += ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
                           (timer_end.tv_usec - timer_start.tv_usec);
            // If this entity is a terrain object, then following code can be skipped.
            // Because properties such as edible, drinkable are nonsense to a terrain object.
            if (isTerrainObject)
                continue;

            if ( moving ) {
                addVectorPredicate( objectNode, AGISIM_VELOCITY_PREDICATE_NAME, tsValue, velocityElement );
            } // if

            Handle agent = AtomSpaceUtil::getAgentHandle( atomSpace, avatarInterface.getPetId( ) );
            if ( agent != objectNode ) { // an agent cannot see himself
                AtomSpaceUtil::setPredicateValue( atomSpace, "inside_pet_fov",
                                                  SimpleTruthValue( (isObjectInsidePetFov ? 1.0f : 0.0f), 1.0f ), agent, objectNode );
            } // if

            addPropertyPredicate(std::string("is_moving"), objectNode, moving);
            addPropertyPredicate(std::string("is_edible"), objectNode, edible, true);
            addPropertyPredicate(std::string("is_drinkable"), objectNode, drinkable, true);
            addPropertyPredicate(std::string("is_toy"), objectNode, isToy, true);

            addInheritanceLink(std::string("pet_home"), objectNode, petHome);
            addInheritanceLink(std::string("food_bowl"), objectNode, foodBowl);
            addInheritanceLink(std::string("water_bowl"), objectNode, waterBowl);

            
            Handle objSemeNode = atomSpace.addNode( SEME_NODE, internalEntityId, SimpleTruthValue( 1, 1 ) );            
            Handle objWordNode = atomSpace.addNode( WORD_NODE, entityClass );
            Handle objClassNode = atomSpace.addNode( CONCEPT_NODE, entityClass, SimpleTruthValue( 1, 1 ) );

            { // each seme node must inherit from its class
                // first normalize the type name to make it
                // compatible with the classes defined at opencog/scm/predicates-frames.scm
                std::string typeName = entityType;
                boost::to_lower(typeName);
                typeName[0] = std::toupper(typeName[0]);
                
                // now create the inheritance link
                Handle objectType = atomSpace.addNode( CONCEPT_NODE, typeName, SimpleTruthValue( 1, 1 ) );
                HandleSeq inheritance(2);
                inheritance[0] = objSemeNode;
                inheritance[1] = objectType;
                
                Handle link = atomSpace.addLink( INHERITANCE_LINK, 
                    inheritance, SimpleTruthValue( 1, 1 ) );
                atomSpace.setLTI( link, 1 );

                HandleSeq classReference(2);
                classReference[0] = objClassNode;
                classReference[1] = objSemeNode;

                Handle referenceLink = 
                    atomSpace.addLink( REFERENCE_LINK, classReference, TruthValue::TRUE_TV( ) );
                atomSpace.setLTI( referenceLink, 1 );

                // finally create a frame for representing this object
                std::map<std::string, Handle> elements;
                elements["Entity"] = objSemeNode;
                elements["Name"] = objClassNode;
                elements["Type"] = objectType;
                AtomSpaceUtil::setPredicateFrameFromHandles( 
                    atomSpace, "#Entity", internalEntityId + "_entity",
                        elements, TruthValue::TRUE_TV() );

                // JaredW: connect the AccessoryNode[etc] directly to the ConceptNode.
                // Sometimes the ObjectNode[subtype] is connected to a (Node "accessory") or similar.
                // It should use the (ConceptNode "Accessory"), as defined above (and only that).
                // Useful for inference and frequent subgraph mining
                {
                    HandleSeq inheritance(2);
                    inheritance[0] = objectNode;
                    inheritance[1] = objectType;

                    Handle link = atomSpace.addLink( INHERITANCE_LINK,
                        inheritance, SimpleTruthValue( 1, 1 ) );
                    atomSpace.setLTI( link, 1 );
                }
            } // end block

            // JaredW: Add e.g. (InheritanceLink (AccessoryNode "id_315840") (ConceptNode "ball"))
            {
                HandleSeq inheritance(2);
                inheritance[0] = objectNode;
                inheritance[1] = objClassNode;

                Handle link = atomSpace.addLink( INHERITANCE_LINK,
                    inheritance, SimpleTruthValue( 1, 1 ) );
                atomSpace.setLTI( link, 1 );
            }
            
            //connect the acessory node to the seme node
            HandleSeq referenceLinkOutgoing1;

            referenceLinkOutgoing1.push_back(objectNode);
            referenceLinkOutgoing1.push_back(objSemeNode);
            {
                Handle referenceLink = atomSpace.addLink( REFERENCE_LINK, referenceLinkOutgoing1, TruthValue::TRUE_TV( ) );
                atomSpace.setLTI( referenceLink, 1 );
            }

            //connect the seme node to the word node
            HandleSeq referenceLinkOutgoing2;
            referenceLinkOutgoing2.push_back(objSemeNode);
            referenceLinkOutgoing2.push_back(objWordNode);
            {
                Handle referenceLink = atomSpace.addLink( REFERENCE_LINK, referenceLinkOutgoing2, TruthValue::TRUE_TV( ) );
                atomSpace.setLTI( referenceLink, 1 );
            }

            //material property
            if ( material.length() > 0 ){
                printf("Material found: %s\n",material.c_str());
                
                Handle materialWordNode = atomSpace.addNode( WORD_NODE, material );
                Handle materialConceptNode = atomSpace.addNode( CONCEPT_NODE, material );

                HandleSeq referenceLinkOutgoing;
                referenceLinkOutgoing.push_back(materialConceptNode);
                referenceLinkOutgoing.push_back(materialWordNode);

                {
                    Handle referenceLink = atomSpace.addLink( REFERENCE_LINK, referenceLinkOutgoing, TruthValue::TRUE_TV( ) );
                    atomSpace.setLTI( referenceLink, 1 );
                }

                AtomSpaceUtil::setPredicateValue( atomSpace, "material",
                                                  SimpleTruthValue( 1.0, 1.0 ), objectNode, materialConceptNode );

            }

             //texture property
            if ( texture.length() ){
                printf("Texture found: %s\n",texture.c_str());
                Handle textureWordNode = atomSpace.addNode( WORD_NODE, texture);
                Handle textureConceptNode = atomSpace.addNode( CONCEPT_NODE, texture);
                HandleSeq referenceLinkOutgoing;
                referenceLinkOutgoing.push_back(textureConceptNode);
                referenceLinkOutgoing.push_back(textureWordNode);
                {
                    Handle referenceLink = atomSpace.addLink( REFERENCE_LINK, referenceLinkOutgoing, TruthValue::TRUE_TV( ));
                    atomSpace.setLTI( referenceLink, 1 );
                }

                AtomSpaceUtil::setPredicateValue( atomSpace, "texture",
                                                  SimpleTruthValue( 1.0f, 1.0f ), objectNode, textureConceptNode );
            }
            
            //color properties
            std::map<std::string, float>::const_iterator it;
            for (it = colors.begin(); it != colors.end(); it++) {
                printf("Color property found: %s (%f)\n", it->first.c_str(), it->second);
                Handle colorWordNode = atomSpace.addNode( WORD_NODE, it->first);
                Handle colorConceptNode = atomSpace.addNode( CONCEPT_NODE, it->first);
                HandleSeq referenceLinkOutgoing;
                referenceLinkOutgoing.push_back(colorConceptNode);
                referenceLinkOutgoing.push_back(colorWordNode);
                {
                    Handle referenceLink = atomSpace.addLink( REFERENCE_LINK, referenceLinkOutgoing, TruthValue::TRUE_TV( ));
                    atomSpace.setLTI( referenceLink, 1 );
                }
                    
                SimpleTruthValue tv( it->second, 1.0 );
                AtomSpaceUtil::setPredicateValue( atomSpace, "color",
                                                  tv, objectNode, colorConceptNode );

                std::map<std::string, Handle> elements;
                elements["Color"] = colorConceptNode;
                elements["Entity"] = objSemeNode;
                AtomSpaceUtil::setPredicateFrameFromHandles( 
                    atomSpace, "#Color", internalEntityId + "_" + it->first + "_color",
                        elements, tv );

            }
        }// if (objRemoval)

        XMLString::release(&entityId);
        XMLString::release(&entityType);
        XMLString::release(&objName);
        XMLString::release(&ownerId);
    }

//    printf("PAI - processMapInfo: consumed %f seconds.\n", 1.0 * elapsed_time/1000000);
    // TODO: Check if this is really needed. It seems ImportanceDecayAgent
    // will eventually remove the atoms that represents the space maps and, this
    // way, these maps will be removed from spaceServer as well through the
    // AtomSpace::atomRemoved() method connected to removal signal of AtomSpace.
    if (!keepPreviousMap) atomSpace.getSpaceServer().cleanupSpaceServer();
    // --

    this->avatarInterface.getCurrentModeHandler( ).handleCommand( "notifyMapUpdate", std::vector<std::string>() );

}

Vector PAI::getVelocityData(DOMElement* velocityElement)
{

    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    XMLString::transcode(X_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* X = XMLString::transcode(velocityElement->getAttribute(tag));
    XMLString::transcode(Y_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* Y = XMLString::transcode(velocityElement->getAttribute(tag));
    XMLString::transcode(Z_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* Z = XMLString::transcode(velocityElement->getAttribute(tag));

    Vector result(atof(X), atof(Y), atof(Z));

    XMLString::release(&X);
    XMLString::release(&Y);
    XMLString::release(&Z);

    return result;
}

Handle PAI::addActionToAtomSpace(ActionPlanID planId, const PetAction& action) throw (opencog::RuntimeException, opencog::InvalidParamException, std::bad_exception)
{

    Handle schemaNode = AtomSpaceUtil::addNode(atomSpace, GROUNDED_SCHEMA_NODE, action.getName().c_str());
    const list<ActionParameter>& params = action.getParameters();
    HandleSeq schemaListLinkOutgoing;
    for (list<ActionParameter>::const_iterator itr = params.begin(); itr != params.end(); itr++) {
        const ActionParameter& param = *itr;

        // Convert the parameter to the corresponding atom representation, depending on its type

        switch (param.getType().getCode()) {
        case BOOLEAN_CODE: {
            const char* value = param.getStringValue().c_str();
            if (strcmp(value, "true") && strcmp(value, "false")) {
                throw opencog::InvalidParamException(TRACE_INFO,
                                                     "PAI - Invalid value for boolean parameter: '%s'. It should be true or false.", value);
            }
            schemaListLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, value));
            break;
        }
        case INT_CODE:
        case FLOAT_CODE: {
            // TODO: check if string is a valid number?
            schemaListLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, param.getStringValue().c_str()));
            break;
        }
        case STRING_CODE: {
            // TODO: converts the string to lowercase?
            schemaListLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, param.getStringValue().c_str()));
            break;
        }
        case VECTOR_CODE: {
            const Vector& value = param.getVectorValue();
            HandleSeq vectorListLink;
            // TODO: check if string is a valid number?
            vectorListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(value.x).c_str()));
            vectorListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(value.y).c_str()));
            vectorListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(value.z).c_str()));
            schemaListLinkOutgoing.push_back(AtomSpaceUtil::addLink(atomSpace, LIST_LINK, vectorListLink));
            break;
        }
        case ROTATION_CODE: {
            const Rotation& value = param.getRotationValue();
            HandleSeq rotationListLink;
            // TODO: check if string is a valid number?
            rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(value.pitch).c_str()));
            rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(value.roll).c_str()));
            rotationListLink.push_back(AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(value.yaw).c_str()));
            schemaListLinkOutgoing.push_back(AtomSpaceUtil::addLink(atomSpace, LIST_LINK, rotationListLink));
            break;
        }
        case ENTITY_CODE: {
            const Entity& value = param.getEntityValue();
            schemaListLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, getSLObjectNodeType(value.type.c_str()), value.id.c_str()));
            break;
        }
        default: {
            // Should never get here, but...
            throw opencog::RuntimeException(TRACE_INFO,
                                            "PAI - Undefined map from '%s' action param type to Atom representation.",
                                            param.getType().getName().c_str());
            break;
        }
        }
    }
    Handle schemaListLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, schemaListLinkOutgoing);

    HandleSeq execLinkOutgoing;
    execLinkOutgoing.push_back(schemaNode);
    execLinkOutgoing.push_back(schemaListLink);
    Handle execLink = AtomSpaceUtil::addLink(atomSpace, EXECUTION_LINK, execLinkOutgoing);

    return execLink;
}

Handle PAI::addActionPredicate(const char* predicateName, const PetAction& action, unsigned long timestamp, ActionID actionId)
{
    Handle execLink = actionId;
    logger().debug("PAI - execLink = %s \n", atomSpace.atomAsString(execLink).c_str());

    HandleSeq predicateListLinkOutgoing;
    predicateListLinkOutgoing.push_back(execLink);
    Handle predicateListLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, predicateListLinkOutgoing);

    Handle predicateNode = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, predicateName);

    HandleSeq evalLinkOutgoing;
    evalLinkOutgoing.push_back(predicateNode);
    evalLinkOutgoing.push_back(predicateListLink);
    Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);

    Handle atTimeLink = atomSpace.getTimeServer().addTimeInfo(evalLink, timestamp);
    AtomSpaceUtil::updateLatestPetActionPredicate(atomSpace, atTimeLink, predicateNode);

    return atTimeLink;
}

Vector PAI::addVectorPredicate(Handle objectNode, const std::string& predicateName, unsigned long timestamp,
                               DOMElement* vectorElement)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    XMLString::transcode(X_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* X = XMLString::transcode(vectorElement->getAttribute(tag));
    XMLString::transcode(Y_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* Y = XMLString::transcode(vectorElement->getAttribute(tag));
    XMLString::transcode(Z_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* Z = XMLString::transcode(vectorElement->getAttribute(tag));

    Handle predNode = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, predicateName, true);

    // TODO: check if string is a valid number?
    Handle xNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, X);
    Handle yNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, Y);
    Handle zNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, Z);

    HandleSeq listLinkOutgoing;
    listLinkOutgoing.push_back(objectNode);
    listLinkOutgoing.push_back(xNode);
    listLinkOutgoing.push_back(yNode);
    listLinkOutgoing.push_back(zNode);
    Handle listLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, listLinkOutgoing);

    HandleSeq evalLinkOutgoing;
    evalLinkOutgoing.push_back(predNode);
    evalLinkOutgoing.push_back(listLink);
    Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);

    Handle atTimeLink = atomSpace.getTimeServer().addTimeInfo( evalLink, timestamp);
    AtomSpaceUtil::updateLatestSpatialPredicate(atomSpace, atTimeLink, predNode, objectNode);

    Vector result(atof(X), atof(Y), atof(Z));

    XMLString::release(&X);
    XMLString::release(&Y);
    XMLString::release(&Z);

    return result;
}

Rotation PAI::addRotationPredicate(Handle objectNode, unsigned long timestamp,
                                   DOMElement* rotationElement)
{
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH+1];

    XMLString::transcode(PITCH_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* pitch = XMLString::transcode(rotationElement->getAttribute(tag));
    XMLString::transcode(ROLL_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* roll = XMLString::transcode(rotationElement->getAttribute(tag));
    XMLString::transcode(YAW_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* yaw = XMLString::transcode(rotationElement->getAttribute(tag));

    Handle predNode = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, AGISIM_ROTATION_PREDICATE_NAME, true);

    // TODO: check if string is a valid number?
    Handle pitchNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, pitch);
    Handle rollNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, roll);
    Handle yawNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, yaw);

    HandleSeq listLinkOutgoing;
    listLinkOutgoing.push_back(objectNode);
    listLinkOutgoing.push_back(pitchNode);
    listLinkOutgoing.push_back(rollNode);
    listLinkOutgoing.push_back(yawNode);
    Handle listLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, listLinkOutgoing);

    HandleSeq evalLinkOutgoing;
    evalLinkOutgoing.push_back(predNode);
    evalLinkOutgoing.push_back(listLink);
    Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);

    Handle atTimeLink = atomSpace.getTimeServer().addTimeInfo( evalLink, timestamp);
    AtomSpaceUtil::updateLatestSpatialPredicate(atomSpace, atTimeLink, predNode, objectNode);

    Rotation result(atof(pitch), atof(roll), atof(yaw));

    XMLString::release(&pitch);
    XMLString::release(&roll);
    XMLString::release(&yaw);

    return result;
}

void PAI::addPropertyPredicate(std::string predicateName, Handle objectNode, bool propertyValue, bool permanent)
{

    SimpleTruthValue tv(0.0, 1.0);

    // if predicate is true set its TV to 1.0, otherwise leave it 0.0
    if (propertyValue) {
        tv.setMean(1.0);
    }

    logger().fine("PAI - Adding predName: %s, value: %d, obj: %s",
                 predicateName.c_str(), (int)propertyValue,
                 atomSpace.getName(objectNode).c_str());

    AtomSpaceUtil::addPropertyPredicate(atomSpace, predicateName, objectNode, tv, permanent);
}

void PAI::addInheritanceLink(std::string conceptNodeName, Handle subNodeHandle, bool inheritanceValue)
{

    if (inheritanceValue) {
        logger().debug("PAI - Inheritance: %s => %s",
                     atomSpace.getName(subNodeHandle).c_str(),
                     conceptNodeName.c_str());

        Handle conceptNodeHandle = AtomSpaceUtil::addNode(atomSpace, CONCEPT_NODE, conceptNodeName.c_str(), true);
        atomSpace.setLTI(subNodeHandle, 1);

        HandleSeq seq;
        seq.push_back(subNodeHandle);
        seq.push_back(conceptNodeHandle);

        Handle link = AtomSpaceUtil::addLink(atomSpace, INHERITANCE_LINK, seq, true);
        atomSpace.setTV(link, SimpleTruthValue(1.0, 1.0));
    }
}

bool PAI::addSpacePredicates(bool keepPreviousMap, Handle objectNode, unsigned long timestamp,
                             DOMElement* positionElement,
                             DOMElement* rotationElement,
                             double length, double width, double height,
                             bool isEdible, bool isDrinkable, bool isTerrainObject)
{

    bool isSelfObject = (avatarInterface.getPetId() == atomSpace.getName(objectNode));

    // Position predicate
    Vector position = addVectorPredicate(objectNode, AGISIM_POSITION_PREDICATE_NAME, timestamp, positionElement);

    // rotation predicate
    Rotation rotation = addRotationPredicate(objectNode, timestamp, rotationElement);


    // Entity class
    std::string entityClass = "common";

    // Size predicate
    if (length > 0 && width > 0 && height > 0) {
        Handle predNode = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, SIZE_PREDICATE_NAME, true);

        Handle lengthNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(length).c_str());
        Handle widthNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(width).c_str());
        Handle heightNode = AtomSpaceUtil::addNode(atomSpace, NUMBER_NODE, opencog::toString(height).c_str());

        HandleSeq listLinkOutgoing;
        listLinkOutgoing.push_back(objectNode);
        listLinkOutgoing.push_back(lengthNode);
        listLinkOutgoing.push_back(widthNode);
        listLinkOutgoing.push_back(heightNode);
        Handle listLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, listLinkOutgoing);

        HandleSeq evalLinkOutgoing;
        evalLinkOutgoing.push_back(predNode);
        evalLinkOutgoing.push_back(listLink);

        // For now, assume any object's size does not change in time.
        // TODO: uncomment this for variable size objects, if any, in the future
        AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);
        //Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);
        //atomSpace.addTimeInfo( evalLink, timestamp);

        // If the object is the Pet, save its radius for new space maps
        if (isSelfObject) {
            agentRadius = sqrt(length * length + width * width) / 2;
            agentHeight = height;
            atomSpace.getSpaceServer().setAgentRadius(agentRadius);
            atomSpace.getSpaceServer().setAgentHeight(agentHeight);
        }
    } else {
        // Get the length, width and height of the object
        // For now, assume any object's size does not change in time.
        logger().warn("PAI - addSpacePredicates(): No size information available in mapinfo.");

        Handle predNode = atomSpace.getHandle( PREDICATE_NODE, SIZE_PREDICATE_NAME);
        if (predNode != Handle::UNDEFINED) {
            HandleSeq sizeEvalLinks;
            HandleSeq outgoing;
            outgoing.push_back(predNode);
            outgoing.push_back(Handle::UNDEFINED);
            atomSpace.getHandleSet(back_inserter(sizeEvalLinks), outgoing, NULL, NULL, 2, EVALUATION_LINK, false);
            foreach(Handle evalLink, sizeEvalLinks) {
                Handle listLink = atomSpace.getOutgoing(evalLink, 1);
                if (atomSpace.getType(listLink) == LIST_LINK && atomSpace.getArity(listLink) == 4) {
                    if (atomSpace.getOutgoing(listLink, 0) == objectNode) {
                        // Found size info for the SL object
                        // TODO: Check for latest (more recent) EvalLink("size",...) from multiple ones, for sizable SL objects,
                        // if any, in the future
                        length = atof(atomSpace.getName(atomSpace.getOutgoing(listLink, 1)).c_str());
                        width = atof(atomSpace.getName(atomSpace.getOutgoing(listLink, 2)).c_str());
                        height = atof(atomSpace.getName(atomSpace.getOutgoing(listLink, 3)).c_str());
                    }
                }
            }
            logger().warn("PAI - addSpacePredicates(): Got last size information available.");
        }
        if (length <= 0 || width <= 0 || height <= 0) {
            logger().warn("PAI - addSpacePredicates(): No size information available for SL object: %s\n", atomSpace.getName(objectNode).c_str());
            if (length < 0) length = 0;
            if (width < 0) width = 0;
            if (height < 0) height = 0;
        }
    }

    // SPACE SERVER INSERTION:
    bool isObstacle;

    if (!isTerrainObject) {
        Handle petHandle = AtomSpaceUtil::getAgentHandle( atomSpace, avatarInterface.getPetId());
        std::string objectName = atomSpace.getName( objectNode );
        bool isAgent = AtomSpaceUtil::getAgentHandle( atomSpace, objectName ) != Handle::UNDEFINED;
        double pet_length, pet_width, pet_height;

        bool hasPetHeight = (petHandle != Handle::UNDEFINED) && AtomSpaceUtil::getSizeInfo(atomSpace, petHandle, pet_length, pet_width, pet_height);

        bool isPickupable = AtomSpaceUtil::isPredicateTrue(atomSpace, "is_pickupable", objectNode);
        isObstacle = !isSelfObject && (isAgent || ((hasPetHeight ? (height > 0.3f * pet_height) : true) && !isPickupable));

        logger().debug("PAI - addSpacePredicates - Adding object to spaceServer. name[%s], isAgent[%s], hasPetHeight[%s], isObstacle[%s], height[%f], pet_height[%f], is_pickupable[%s], isSelfObject[%s]", objectName.c_str( ), (isAgent ? "t" : "f"), (hasPetHeight ? "t" : "f"), (isObstacle ? "t" : "f"), height, pet_height, (isPickupable ? "t" : "f"), (isSelfObject ? "t" : "f") );
    } else {
        isObstacle = true;
        entityClass = "terrain_block";
    }


    return atomSpace.getSpaceServer().addSpaceInfo(keepPreviousMap, objectNode, timestamp, position.x, position.y, position.z, length, width, height, rotation.yaw, isObstacle, entityClass);
}

Handle PAI::addPhysiologicalFeeling(const string petID,
                                    const string feeling,
                                    unsigned long timestamp,
                                    float level)
{
    HandleSeq evalLinkOutgoing;

    Handle feelingNode = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, feeling);

    //! @todo Can only be a "Pet". But we're going to merge the Pet and Humanoid agent-types anyway
    Handle agentNode = AtomSpaceUtil::addNode(atomSpace, PET_NODE, petID);

    // Add EvaluationLink
    evalLinkOutgoing.push_back(feelingNode);
    HandleSeq dummy;dummy.push_back(agentNode);
    evalLinkOutgoing.push_back(AtomSpaceUtil::addLink(atomSpace, LIST_LINK, dummy));
    Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);

    // Time stamp the EvaluationLink
    //
    // AtTimeLink
    //     TimeNode "timestamp"
    //     EvaluationLink
    //
    Handle atTimeLink = atomSpace.getTimeServer().addTimeInfo(evalLink, timestamp);

    // count=1, i.e. one observation of this biological urge
    atomSpace.setTV(atTimeLink,SimpleTruthValue((strength_t)level, 1));

    // TODO: what does this for? [by Zhenhua Cai, on 2011-03-08]
    AtomSpaceUtil::updateLatestPhysiologicalFeeling(atomSpace, atTimeLink, feelingNode);
    
    // setup the frame for the given physiological feeling

    // Create the name for the frame
    string frameName(feeling);
    boost::replace_first(frameName, "_urgency", "");
    std::string frameInstanceName = avatarInterface.getPetId() + "_" + frameName + "_biological_urge";

    if (level > 0) {
        std::string degree = (level >= 0.7) ?
                                 "High" :
                                 (level >= 0.3) ?
                                     "Medium" : 
                                     "Low";

        std::map<std::string, Handle> elements;
        elements["Experiencer"] = atomSpace.addNode( SEME_NODE, avatarInterface.getPetId() );
        elements["State"] = atomSpace.addNode( CONCEPT_NODE, frameName );
        elements["Degree"] = atomSpace.addNode( CONCEPT_NODE, degree );
        elements["Value"] = atomSpace.addNode( NUMBER_NODE, boost::lexical_cast<std::string>( level ) );

        AtomSpaceUtil::setPredicateFrameFromHandles( atomSpace,
                                                     "#Biological_urge", 
                                                     frameInstanceName,
                                                     elements, 
                                                     SimpleTruthValue( (level < 0.5) ? 0.0 : level, 1.0 )
                                                    );
    } else {
        Handle predicateNode = atomSpace.getHandle(PREDICATE_NODE, frameInstanceName);
        if ( predicateNode != Handle::UNDEFINED ) {
            AtomSpaceUtil::deleteFrameInstance(atomSpace, feelingNode);
        } // if
    } // else
    
    return evalLink;
}

Handle PAI::addOwnershipRelation(const Handle owner, const Handle owned, bool isPetObject)
{
    HandleSeq listLinkOutgoing;
    listLinkOutgoing.push_back(owner);
    listLinkOutgoing.push_back(owned);

    // NOTE: if owned is the pet itself, this relation cannot be forgotten
    HandleSeq evalLinkOutgoing;
    evalLinkOutgoing.push_back(AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, OWNERSHIP_PREDICATE_NAME, isPetObject));
    evalLinkOutgoing.push_back(AtomSpaceUtil::addLink(atomSpace, LIST_LINK, listLinkOutgoing, isPetObject));
    Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing, isPetObject);

    // set predicate evaluation true
    atomSpace.setTV(evalLink, SimpleTruthValue(1.0f, 0.0f));

    // No time info for now - UNCOMMENT if ownership needs time info
    //atomSpace.addTimeInfo( evalLink, timestamp);

    return evalLink;
}

//TODO: THIS METHOD IS NOT BEING CALLED AT ALL. THIS MEANS THAT, IF PVP DO NOT SEND ANY STATUS FOR THESE PLANS,
//THEY WILL BE IN INTERNAL DATA STRUCTURES FOREVER. SUGGESTION: CALL THIS METHOD WHEN A SCHEMA TIMEOUT HAPPENS IN SCHEMARUNNER
void PAI::setPendingActionPlansFailed()
{

    ActionPlanMap::iterator it;
    for (it =  pendingActionPlans.begin(); it != pendingActionPlans.end(); it++) {
        ActionPlanID planId = it->first;

        setActionPlanStatus(planId, 0, opencog::pai::ERROR,
                            getLatestSimWorldTimestamp());
    }
}

void PAI::setActionPlanStatus(ActionPlanID& planId, unsigned int sequence,
                              ActionStatus statusCode, unsigned long timestamp)
{

    // Get the corresponding actions from pending map and add the given perceptions
    // into AtomSpace
    ActionPlanMap::iterator it = pendingActionPlans.find(planId);
    if (it != pendingActionPlans.end()) {
        ActionPlan& plan = it->second;

        set<unsigned int> seqNumbers;
        if (sequence != 0) {
            seqNumbers.insert(sequence);
            // TODO: Check for string->int conversion failures

        } else {
            // this statusCode is for all actions in the action plan
            // that were not marked as done or failed yet
            for (unsigned int i = 1; i <= plan.size(); i++) {
                if (!plan.isDone(i) && !plan.isFailed(i)) {
                    seqNumbers.insert(i);
                }
            }
        }

        set<unsigned int>::const_iterator itr;
        for (itr = seqNumbers.begin(); itr != seqNumbers.end(); itr++) {
            unsigned int seqNumber = *itr;
            const char* predicateName;

            switch (statusCode) {
            case opencog::pai::DONE:
                {
                    const PetAction& action = plan.getAction(seqNumber);

                    // if a GRAB action, set grabbed object
                    if (action.getType() == ActionType::GRAB()) {
                        const std::list<ActionParameter>& params = action.getParameters();

                        foreach(ActionParameter param, params) {
                            if (param.getName() == "target") {
                                const Entity& entity = param.getEntityValue();
                                AtomSpaceUtil::setupHoldingObject(  atomSpace,
                                                                    avatarInterface.getPetId( ),
                                                                    entity.id,
                                                                    getLatestSimWorldTimestamp() );
                                avatarInterface.setGrabbedObj(entity.id);
                            }
                        }
                    }
                    // if a DROP or NUDGE_TO action unset grabbed object
                    else if (action.getType() == ActionType::DROP() ||
                             action.getType() == ActionType::NUDGE_TO()) {
                        AtomSpaceUtil::setupHoldingObject(  atomSpace,
                                                            avatarInterface.getPetId( ), "",
                                                            getLatestSimWorldTimestamp() );
                        avatarInterface.setGrabbedObj(std::string(""));
                    }

                    plan.markAsDone(seqNumber);
                    predicateName = ACTION_DONE_PREDICATE_NAME;
                    break;
                }
            case opencog::pai::ERROR:
                plan.markAsFailed(seqNumber);
                predicateName = ACTION_FAILED_PREDICATE_NAME;
                failedActionPlans.insert(planId);
                // TODO: Log the value of the parameter with name="reason"
                cout << "PAI Action plan FAILED: " << plan.getID() << endl;
                break;
            default:
                break;
            }
            ActionID actionHandle = planToActionIdsMaps[planId][seqNumber];
            //printf("calling addActionPredicate for action with seqNumber = %u\n", seqNumber);
            //! @todo
            assert(atomSpace.isValidHandle(actionHandle));
            addActionPredicate(predicateName, plan.getAction(seqNumber), timestamp, actionHandle);
        }

        // If there is no pending actions anymore
        if (plan.isFinished()) {
            cout << "PAI Action plan OK: " << plan.getID() << endl;
            pendingActionPlans.erase(planId);
            planToActionIdsMaps.erase(planId);
        } else {
			// If using loosely ordered action sending mode,
			// then continue to send the next action of this
			// unfinished plan.
			if(config().get_bool("EXTRACTED_ACTION_MODE")) {
				if(sequence != 0) {
					int nextActionSeqNum = sequence + 1;
					sendExtractedActionFromPlan(planId, nextActionSeqNum);
				}
			}
		} 

    } else {
        logger().warn(
                     "PAI - No pending action plan with the given id '%s'.",
                     planId.c_str());
    }
}

std::string PAI::camelCaseToUnderscore(std::string s)
{
    for (string::iterator it = s.begin(); it != s.end(); ++it)
        if (*it >= 'A' && *it <= 'Z') {
            *it = tolower(*it);
            it = s.insert(it, '_');
            ++it;
        }
    return s;
}

std::string PAI::camelCaseToUnderscore(const char* s)
{
    string str(s);
    return camelCaseToUnderscore(str);
}
