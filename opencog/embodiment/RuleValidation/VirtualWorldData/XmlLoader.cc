/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/XmlLoader.cc
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

#include "XmlLoader.h"

#include "VirtualWorldXmlConstants.h"

#include <xercesc/dom/DOM.hpp>
#include <xercesc/framework/LocalFileInputSource.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <opencog/util/files.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>

using namespace VirtualWorldData;
using namespace opencog;

XmlLoader::XmlLoader()
{
    initializeXMLPlatform();

    this->parser = new XERCES_CPP_NAMESPACE::XercesDOMParser();
    this->parser->setErrorHandler(&errorHandler);

    // The following lines enable validation
    parser->cacheGrammarFromParse(true);

    // only when setDoSchema(true) is called.
    parser->setValidationScheme(XERCES_CPP_NAMESPACE::XercesDOMParser::Val_Auto);
    parser->setDoNamespaces(true);
    parser->setDoSchema(true);

#define XSD_NAMESPACE "http://www.opencog.org/embodiment"
#define XSD_FILE_NAME "VirtualWorld.xsd"

    if (fileExists(XSD_FILE_NAME)) {
        std::string schemaLocation = XSD_NAMESPACE " " XSD_FILE_NAME;
        // The line bellow replace the path for the XSD file so that it does not
        // need to be in the current directory...
        parser->setExternalSchemaLocation(schemaLocation.c_str());
    }
}

XmlLoader::~XmlLoader()
{
    delete parser;
    terminateXMLPlatform();
}

void XmlLoader::initializeXMLPlatform()
{

    // Initialize the XML system
    try {
        XERCES_CPP_NAMESPACE::XMLPlatformUtils::Initialize();
    } catch (const XERCES_CPP_NAMESPACE::XMLException &toCatch) {
    }
}

void XmlLoader::terminateXMLPlatform()
{
    XERCES_CPP_NAMESPACE::XMLPlatformUtils::Terminate();
}

void XmlLoader::processWorldStateDocument(XERCES_CPP_NAMESPACE::DOMDocument * doc,
        VirtualWorldState & worldState)
{
    XMLCh tag[XML_TAG_LENGTH + 1];
    XERCES_CPP_NAMESPACE::DOMNodeList * list;

    // getting <entity-info> elements from the XML message
    XERCES_CPP_NAMESPACE::XMLString::transcode(ENTITY_INFO_ELEM, tag, XML_TAG_LENGTH);
    list = doc->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        processEntityInfo((XERCES_CPP_NAMESPACE::DOMElement *)list->item(i), worldState);
    }

    // getting <indefinite-object-info> elements from the XML message
    XERCES_CPP_NAMESPACE::XMLString::transcode(INDEF_OBJ_ELEM, tag, XML_TAG_LENGTH);
    list = doc->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        processIndefiniteObjectInfo((XERCES_CPP_NAMESPACE::DOMElement *)list->item(i), worldState);
    }

    // getting <world-state-info> elements from the XML message
    XERCES_CPP_NAMESPACE::XMLString::transcode(WORLD_STATE_INFO_ELEM, tag, XML_TAG_LENGTH);
    list = doc->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        processWorldStateInfo((XERCES_CPP_NAMESPACE::DOMElement *)list->item(i), worldState);
    }
}

void XmlLoader::processEntityInfo(XERCES_CPP_NAMESPACE::DOMElement * element,
                                  VirtualWorldState & worldState )
{
    XMLCh tag[XML_TAG_LENGTH + 1];
    XERCES_CPP_NAMESPACE::DOMNodeList * list;

    // getting <entity-object> elements from the XML message
    XERCES_CPP_NAMESPACE::XMLString::transcode(ENTITY_OBJ_ELEM, tag, XML_TAG_LENGTH);
    list = element->getElementsByTagName(tag);
    logger().debug("XmlLoader - Number of entity founds '%d'.",
                 list->getLength());

    for (unsigned int i = 0; i < list->getLength(); i++) {
        VirtualEntity entity;
        processEntityElement((XERCES_CPP_NAMESPACE::DOMElement *)list->item(i), entity);

        logger().debug("XmlLoader - Entity to be added '%s'.",
                     entity.getName().c_str());
        worldState.addEntity(entity);
    }

    // getting <agent-object> elements from the XML message
    XERCES_CPP_NAMESPACE::XMLString::transcode(AGENT_OBJ_ELEM, tag, XML_TAG_LENGTH);
    list = element->getElementsByTagName(tag);
    logger().debug("XmlLoader - Number of agents found '%d'.",
                 list->getLength());

    for (unsigned int i = 0; i < list->getLength(); i++) {
        VirtualAgent agent;
        processEntityElement((XERCES_CPP_NAMESPACE::DOMElement *)list->item(i), agent);
        processAgentElement((XERCES_CPP_NAMESPACE::DOMElement *)list->item(i), agent);

        logger().debug("XmlLoader - Agent to be added '%s'.",
                     agent.getName().c_str());
        worldState.addEntity(agent);
    }
}

void XmlLoader::processEntityElement(XERCES_CPP_NAMESPACE::DOMElement * element,
                                     VirtualWorldData::VirtualEntity & entity)
{
    bool result;
    char * attribute;

    XMLCh tag[XML_TAG_LENGTH + 1];

    // getting name
    XERCES_CPP_NAMESPACE::XMLString::transcode(NAME_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    entity.setName(std::string(attribute));
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting type
    XERCES_CPP_NAMESPACE::XMLString::transcode(TYPE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    if (strcmp(attribute, "pet") == 0) {
        entity.setType(VirtualWorldData::PET);
    } else if (strcmp(attribute, "avatar") == 0) {
        entity.setType(VirtualWorldData::AVATAR);
    } else if (strcmp(attribute, "") != 0) {
        entity.setType(VirtualWorldData::OBJECT);
    }
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting noisy property
    XERCES_CPP_NAMESPACE::XMLString::transcode(NOISY_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setNoisy(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting small property
    XERCES_CPP_NAMESPACE::XMLString::transcode(SMALL_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setSmall(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting edible property
    XERCES_CPP_NAMESPACE::XMLString::transcode(EDIBLE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setEdible(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting movable property
    XERCES_CPP_NAMESPACE::XMLString::transcode(MOVABLE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setMovable(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting drinkable property
    XERCES_CPP_NAMESPACE::XMLString::transcode(DRINKABLE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setDrinkable(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting pee place property
    XERCES_CPP_NAMESPACE::XMLString::transcode(PEE_PLACE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setPeePlace(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting poo place property
    XERCES_CPP_NAMESPACE::XMLString::transcode(POO_PLACE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setPooPlace(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting pickupable property
    XERCES_CPP_NAMESPACE::XMLString::transcode(PICKUPABLE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setPickupable(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting moving property
    XERCES_CPP_NAMESPACE::XMLString::transcode(MOVING_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    entity.setMoving(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
}

void XmlLoader::processAgentElement(XERCES_CPP_NAMESPACE::DOMElement * element,
                                    VirtualWorldData::VirtualAgent & agent)
{
    bool result;
    char * attribute;
    XERCES_CPP_NAMESPACE::DOMElement * feelingElement;
    XMLCh tag[XML_TAG_LENGTH + 1];

    // getting novelty property
    XERCES_CPP_NAMESPACE::XMLString::transcode(NOVELTY_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    agent.setNovelty(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting learning property
    XERCES_CPP_NAMESPACE::XMLString::transcode(LEARNING_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    agent.setLearning(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting asked to try property
    XERCES_CPP_NAMESPACE::XMLString::transcode(ASKED_TO_TRY_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    agent.setAvatarAskedToTry(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting learned tricks property
    XERCES_CPP_NAMESPACE::XMLString::transcode(LEARNED_TRICKS_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    agent.setLearnedTricks(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting requested schema property
    XERCES_CPP_NAMESPACE::XMLString::transcode(REQUESTED_SCHEMA_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    result = (strcmp(attribute, "true") == 0 ? true : false);
    agent.setRequestedSchema(result);
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

    // getting hunger physiological feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(HUNGER_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setHunger(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting thirst physiological feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(THIRST_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setThirst(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting energy physiological feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(ENERGY_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setEnergy(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting fitness physiological feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(FITNESS_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setFitness(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting poo urgency physiological feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(POO_URGENCY_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setPooUrgency(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting pee urgency physiological feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(PEE_URGENCY_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setPeeUrgency(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting fear emotional feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(FEAR_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setFear(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting hate emotional feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(HATE_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setHate(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting love emotional feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(LOVE_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setLove(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting anger emotional feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(ANGER_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setAnger(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting pride emotional feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(PRIDE_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setPride(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting gratitude emotional feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(GRATITUDE_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setGratitude(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting happiness emotional feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(HAPPINESS_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setHappiness(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting excitement emotional feeling
    XERCES_CPP_NAMESPACE::XMLString::transcode(EXCITEMENT_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setExcitement(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting aggressiveness trait
    XERCES_CPP_NAMESPACE::XMLString::transcode(AGGRESSIVENESS_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setAggressiveness(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting playfulness trait
    XERCES_CPP_NAMESPACE::XMLString::transcode(PLAYFULNESS_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setPlayfulness(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting curiosity trait
    XERCES_CPP_NAMESPACE::XMLString::transcode(CURIOSITY_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setCuriosity(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting friendliness trait
    XERCES_CPP_NAMESPACE::XMLString::transcode(FRIENDLINESS_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setFriendliness(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting fearfulness trait
    XERCES_CPP_NAMESPACE::XMLString::transcode(FEARFULNESS_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setFearfulness(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting appreciativeness trait
    XERCES_CPP_NAMESPACE::XMLString::transcode(APPRECIATIVENESS_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setAppreciativeness(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }

    // getting excitability trait
    XERCES_CPP_NAMESPACE::XMLString::transcode(EXCITABILITY_ELEM, tag, XML_TAG_LENGTH);
    feelingElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (feelingElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(feelingElement->getAttribute(tag));
        agent.setExcitability(atof(attribute));
        XERCES_CPP_NAMESPACE::XMLString::release(&attribute);
    }
}

void XmlLoader::processIndefiniteObjectInfo(XERCES_CPP_NAMESPACE::DOMElement * element,
        VirtualWorldState & worldState)
{
    char * object;
    XMLCh tag[XML_TAG_LENGTH + 1];
    XERCES_CPP_NAMESPACE::DOMElement * indefiniteElement;
    IndefiniteObjects & indefiniteObjects = worldState.getIndefiniteObjects();

    // nearest-object
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_OBJ_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_object = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-edible
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_EDIBLE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_edible = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-drinkable
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_DRINKABLE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_drinkable = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-movable
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_MOVABLE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_movable = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-pickupable
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_PICKUPABLE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_pickupable = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-avatar
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_AVATAR_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_avatar = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-pet
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_PET_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_pet = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-small
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_SMALL_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_small = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-moving
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_MOVING_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_moving = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-noisy
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_NOISY_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_noisy = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-friendly
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_FRIENDLY_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_friendly = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-poo-place
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_POO_PLACE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_poo_place = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // nearest-pee-place
    XERCES_CPP_NAMESPACE::XMLString::transcode(N_PEE_PLACE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.nearest_pee_place = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-object
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_OBJ_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_object = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-edible
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_EDIBLE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_edible = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-drinkable
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_DRINKABLE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_drinkable = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-movable
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_MOVABLE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_movable = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-pickupable
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_PICKUPABLE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_pickupable = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-avatar
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_AVATAR_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_avatar = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-pet
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_PET_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_pet = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-small
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_SMALL_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_small = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-moving
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_MOVING_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_moving = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-noisy
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_NOISY_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_noisy = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-friendly
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_FRIENDLY_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_friendly = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-poo-place
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_POO_PLACE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_poo_place = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // random-pee-place
    XERCES_CPP_NAMESPACE::XMLString::transcode(R_PEE_PLACE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.random_pee_place = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // food-bowl
    XERCES_CPP_NAMESPACE::XMLString::transcode(FOOD_BOWL_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.food_bowl = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // water-bowl
    XERCES_CPP_NAMESPACE::XMLString::transcode(WATER_BOWL_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.water_bowl = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // pet-bowl
    XERCES_CPP_NAMESPACE::XMLString::transcode(PET_BOWL_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.pet_bowl = std::string(object);

        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // pet-home
    XERCES_CPP_NAMESPACE::XMLString::transcode(PET_HOME_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.pet_home = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // last-food-place
    XERCES_CPP_NAMESPACE::XMLString::transcode(LAST_FOOD_PLACE_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.last_food_place = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }

    // exemplar-avatar
    XERCES_CPP_NAMESPACE::XMLString::transcode(EXEMPLAR_AVATAR_ELEM, tag, XML_TAG_LENGTH);
    indefiniteElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (indefiniteElement != NULL) {
        XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
        object = XERCES_CPP_NAMESPACE::XMLString::transcode(indefiniteElement->getAttribute(tag));
        indefiniteObjects.exemplar_avatar = std::string(object);
        XERCES_CPP_NAMESPACE::XMLString::release(&object);
    }
}

void XmlLoader::processWorldStateInfo(XERCES_CPP_NAMESPACE::DOMElement * element,
                                      VirtualWorldState & worldState)
{

    char * attribute;
    XMLCh tag[XML_TAG_LENGTH + 1];
    XERCES_CPP_NAMESPACE::DOMNodeList * list = NULL;
    XERCES_CPP_NAMESPACE::DOMElement  * predicateElement = NULL;

//    logger().debug("XmlLoader - Processing pet-id.");
    // getting pet-id attribute
    XERCES_CPP_NAMESPACE::XMLString::transcode(PET_ID_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    worldState.setPetId(std::string(attribute));
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

//    logger().debug("XmlLoader - Processing owner-id.");
    // getting owner-id attribute
    XERCES_CPP_NAMESPACE::XMLString::transcode(OWNER_ID_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    worldState.setPetOwnerId(std::string(attribute));
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

//    logger().debug("XmlLoader - Processing pet-mode.");
    // getting pet-mode attribute
    XERCES_CPP_NAMESPACE::XMLString::transcode(PET_MODE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    worldState.setPetMode(std::string(attribute));
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

//    logger().debug("XmlLoader - Processing agent-state.");
    // getting agent-state attribute
    XERCES_CPP_NAMESPACE::XMLString::transcode(AGENT_STATE_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    worldState.setAgentState(atoi(attribute));
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

//    logger().debug("XmlLoader - Processing action-repetition.");
    // getting action-repetition attribute
    XERCES_CPP_NAMESPACE::XMLString::transcode(ACTION_REPETITION_ATTR, tag, XML_TAG_LENGTH);
    attribute = XERCES_CPP_NAMESPACE::XMLString::transcode(element->getAttribute(tag));
    worldState.setCurrentActionRepetition(atoi(attribute));
    XERCES_CPP_NAMESPACE::XMLString::release(&attribute);

//    logger().debug("XmlLoader - Processing near-info.");
    // <near-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(NEAR_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <near> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(NEAR_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * first;
            char * second;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("first", tag, XML_TAG_LENGTH);
            first = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("second", tag, XML_TAG_LENGTH);
            second = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            worldState.addNearObjects(std::string(first), std::string(second));

            XERCES_CPP_NAMESPACE::XMLString::release(&first);
            XERCES_CPP_NAMESPACE::XMLString::release(&second);
        }
    }

//    logger().debug("XmlLoader - Processing next-info.");
    // <next-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(NEXT_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <next> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(NEXT_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * first;
            char * second;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("first", tag, XML_TAG_LENGTH);
            first = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("second", tag, XML_TAG_LENGTH);
            second = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            worldState.addNextObjects(std::string(first), std::string(second));
            XERCES_CPP_NAMESPACE::XMLString::release(&first);
            XERCES_CPP_NAMESPACE::XMLString::release(&second);
        }
    }

//    logger().debug("XmlLoader - Processing owner-info.");
    // <owner-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(OWNER_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <next> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(OWNS_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * first;
            char * second;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("first", tag, XML_TAG_LENGTH);
            first = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("second", tag, XML_TAG_LENGTH);
            second = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            worldState.addOwnerObjects(std::string(first), std::string(second));
            XERCES_CPP_NAMESPACE::XMLString::release(&first);
            XERCES_CPP_NAMESPACE::XMLString::release(&second);
        }
    }

//    logger().debug("XmlLoader - Processing moving-toward.");
    // <moving_toward-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(MOVING_TOWARD_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <moving-toward> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(MOVING_TOWARD_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * first;
            char * second;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("first", tag, XML_TAG_LENGTH);
            first = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("second", tag, XML_TAG_LENGTH);
            second = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            worldState.addMovingTowardObjects(std::string(first), std::string(second));
            XERCES_CPP_NAMESPACE::XMLString::release(&first);
            XERCES_CPP_NAMESPACE::XMLString::release(&second);
        }
    }

//    logger().debug("XmlLoader - Processing inside-fov-info.");
    // <inside_fov-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(INSIDE_FOV_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <inside-fov> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(INSIDE_FOV_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * first;
            char * second;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("first", tag, XML_TAG_LENGTH);
            first = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("second", tag, XML_TAG_LENGTH);
            second = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            worldState.addInsideFoVObjects(std::string(first), std::string(second));
            XERCES_CPP_NAMESPACE::XMLString::release(&first);
            XERCES_CPP_NAMESPACE::XMLString::release(&second);
        }
    }

//    logger().debug("XmlLoader - Processing relations-info.");
    // <relations-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(RELATIONS_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <relation> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(RELATION_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * first;
            char * second;
            char * relation;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("name", tag, XML_TAG_LENGTH);
            relation = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("first", tag, XML_TAG_LENGTH);
            first = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("second", tag, XML_TAG_LENGTH);
            second = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            worldState.addRelationObjects(std::string(first), std::string(second), std::string(relation));
            XERCES_CPP_NAMESPACE::XMLString::release(&first);
            XERCES_CPP_NAMESPACE::XMLString::release(&second);
            XERCES_CPP_NAMESPACE::XMLString::release(&relation);
        }
    }

//    logger().debug("XmlLoader - Processing has-said-info.");
    // <has-said-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(HAS_SAID_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <has-said> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(HAS_SAID_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * first;
            char * message;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("message", tag, XML_TAG_LENGTH);
            message = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("agent", tag, XML_TAG_LENGTH);
            first = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            worldState.addHasSaidObjects(std::string(first), std::string(message));
            XERCES_CPP_NAMESPACE::XMLString::release(&first);
            XERCES_CPP_NAMESPACE::XMLString::release(&message);
        }
    }

//    logger().debug("XmlLoader - Processing last-agent-action-info.");
    // <last-agent-action-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(LAST_AGENT_ACTION_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <agent-action> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(AGENT_ACTION_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * agent;
            char * action;
            XERCES_CPP_NAMESPACE::DOMNodeList * paramList;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("action", tag, XML_TAG_LENGTH);
            action = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("agent", tag, XML_TAG_LENGTH);
            agent = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            std::vector<std::string> parameters;

            // getting <agent-action> elements
            XERCES_CPP_NAMESPACE::XMLString::transcode("param", tag, XML_TAG_LENGTH);
            paramList = genericElement->getElementsByTagName(tag);
            for (unsigned int j = 0; j < list->getLength(); j++) {
                char * param;

                XERCES_CPP_NAMESPACE::DOMElement * paramElement =
                    (XERCES_CPP_NAMESPACE::DOMElement *)paramList->item(j);

                XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
                param = XERCES_CPP_NAMESPACE::XMLString::transcode(paramElement->getAttribute(tag));
                parameters.push_back(std::string(param));

                XERCES_CPP_NAMESPACE::XMLString::release(&param);
            }

            worldState.addLastAgentAction(std::string(agent), std::string(action), parameters);
            XERCES_CPP_NAMESPACE::XMLString::release(&agent);
            XERCES_CPP_NAMESPACE::XMLString::release(&action);
        }
    }

//    logger().debug("XmlLoader - Processing last-pet-schema.");
    // <last-pet-schema-info>
    XERCES_CPP_NAMESPACE::XMLString::transcode(LAST_PET_SCHEMA_INFO_ELEM, tag, XML_TAG_LENGTH);
    predicateElement = (XERCES_CPP_NAMESPACE::DOMElement *)element->getElementsByTagName(tag)->item(0);
    if (predicateElement != NULL) {

        // getting <pet-schema> elements
        XERCES_CPP_NAMESPACE::XMLString::transcode(PET_SCHEMA_ELEM, tag, XML_TAG_LENGTH);
        list = predicateElement->getElementsByTagName(tag);
        for (unsigned int i = 0; i < list->getLength(); i++) {
            char * schema;
            char * result;
            XERCES_CPP_NAMESPACE::DOMNodeList * paramList;

            XERCES_CPP_NAMESPACE::DOMElement * genericElement =
                (XERCES_CPP_NAMESPACE::DOMElement *)list->item(i);

            XERCES_CPP_NAMESPACE::XMLString::transcode("schema", tag, XML_TAG_LENGTH);
            schema = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));
            XERCES_CPP_NAMESPACE::XMLString::transcode("result", tag, XML_TAG_LENGTH);
            result = XERCES_CPP_NAMESPACE::XMLString::transcode(genericElement->getAttribute(tag));

            std::vector<std::string> parameters;

            // getting <pet-schema> elements
            XERCES_CPP_NAMESPACE::XMLString::transcode("param", tag, XML_TAG_LENGTH);
            paramList = genericElement->getElementsByTagName(tag);
            for (unsigned int j = 0; j < list->getLength(); j++) {
                char * param;

                XERCES_CPP_NAMESPACE::DOMElement * paramElement =
                    (XERCES_CPP_NAMESPACE::DOMElement *)paramList->item(j);

                XERCES_CPP_NAMESPACE::XMLString::transcode("value", tag, XML_TAG_LENGTH);
                param = XERCES_CPP_NAMESPACE::XMLString::transcode(paramElement->getAttribute(tag));
                parameters.push_back(std::string(param));

                XERCES_CPP_NAMESPACE::XMLString::release(&param);
            }

            worldState.addLastPetSchema(std::string(schema), std::string(result), parameters);
            XERCES_CPP_NAMESPACE::XMLString::release(&schema);
            XERCES_CPP_NAMESPACE::XMLString::release(&result);
        }
    }
}

/* -----------------------------------------------------------------------------
 * Public methods
 * -----------------------------------------------------------------------------
 */
bool XmlLoader::fromFile(const std::string & filename, VirtualWorldState & worldState)
{

    XMLCh * nameStr = XERCES_CPP_NAMESPACE::XMLString::transcode(filename.c_str());
    XERCES_CPP_NAMESPACE::LocalFileInputSource * fileBufIS =
        new XERCES_CPP_NAMESPACE::LocalFileInputSource(nameStr);

    parser->resetDocumentPool();

    try {
        parser->parse(*fileBufIS);
    } catch (const XERCES_CPP_NAMESPACE::XMLException& toCatch) {
        char * message = XERCES_CPP_NAMESPACE::XMLString::transcode(toCatch.getMessage());
        logger().error("XmlLoader - XML Exception: %s", message);
        XERCES_CPP_NAMESPACE::XMLString::release(&message);
        delete fileBufIS;
        return false;
    } catch (const XERCES_CPP_NAMESPACE::DOMException& toCatch) {
        char* message = XERCES_CPP_NAMESPACE::XMLString::transcode(toCatch.msg);
        logger().error("XmlLoader - DOM Exception: %s", message);
        XERCES_CPP_NAMESPACE::XMLString::release(&message);
        delete fileBufIS;
        return false;
    } catch (...) {
        logger().error("XmlLoader - Unexpected XML Parse Exception");
        delete fileBufIS;
        return false;
    }

    XERCES_CPP_NAMESPACE::DOMDocument * document = NULL;
    if (parser->getErrorCount() == 0) {
        document = parser->adoptDocument();

        try {
            processWorldStateDocument(document, worldState);
//            logger().debug("XmlLoader - processPVPDocument done");

            // catch runtime exceptions and its specialization
        } catch (opencog::RuntimeException& e) {
            delete fileBufIS;
            delete document;
            return false;

        } catch (...) {
            logger().error(
                         "XmlLoader - Got an unknown exception while processing from XML file.");
            delete fileBufIS;
            delete document;
            return false;
        }
    } else {
        logger().error(
                     "XmlLoader - Got %d errors parsing the xml data.", parser->getErrorCount());
        delete fileBufIS;
        delete document;
        return false;
    }

    delete fileBufIS;
    delete document;
    return true;
}
