/*
 * opencog/embodiment/PetaverseProxySimulator/PVPSimulator.cc
 *
 * Copyright (C) 2007-2008 Andre Senna
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

#include "PVPSimulator.h"

#include <iostream>
#include <fstream>

#include <opencog/util/files.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PetAction.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAIUtils.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PetaverseDOMParser.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PetaverseErrorHandler.h>
#include <opencog/embodiment/Control/MessagingSystem/StringMessage.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PVPXmlConstants.h>
#include "SimulationConfig.h"
#include "AGISimSimulator.h"

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace PetaverseProxySimulator;
using namespace AutomatedSystemTest;
using namespace PerceptionActionInterface;
using namespace MessagingSystem;

const int PVPSimulator::PET_SIGNAL_SENDING_INTERVAL = 1; // ticks
const char* PVPSimulator::DEFAULT_PET_ID = "Fido";
const char* PVPSimulator::DEFAULT_OWNER_ID = "Wynx";

opencog::BaseServer* PVPSimulator::createInstance()
{
    return new PVPSimulator;
}

PVPSimulator::~PVPSimulator()
{

    for ( PetsPhysiologicalModelMap::iterator it = petsPhysiologicalModel.begin(); it != petsPhysiologicalModel.end(); it++) {
        delete(it->second);
    }
    petsPhysiologicalModel.clear();

    delete(this->worldSimulator);
    if (opencog::config().get_bool("GENERATE_GOLD_STANDARD")) {
        delete goldStdGen;
    }
}

PVPSimulator::PVPSimulator()
{
}

void PVPSimulator::init()
{
    initialize();
    worldSimulator = new WorldSimulator();
}

void PVPSimulator::init(const std::string &myId, const std::string &ip, int portNumber)
{
    setNetworkElement(new NetworkElement(myId, ip, portNumber));
    PVP_ID = "PVP";
    initialize();
    worldSimulator = new AGISimSimulator("localhost", this);
    //worldSimulator = new AGISimSimulator("10.1.0.5", this);
}

void PVPSimulator::initialize()
{
    PerceptionActionInterface::PAIUtils::initializeXMLPlatform();

    firstMessageFlag = true;
    tickCount = 0;

    messagesToSend.createQueue(PVP_ID);
    pthread_mutex_init(&currentTimeLock, NULL);

    if (opencog::config().get_bool("GENERATE_GOLD_STANDARD")) {
        goldStdGen = new GoldStdGen(opencog::config().get("GOLD_STANDARD_FILENAME").c_str());
    }

    /**
     * Recovery operation (if needed). First mount the filename full path and
     * than check if such file exists in filesystem. If so, start recovery
     * process.
     */
    std::string recoveryFile = opencog::config().get("PROXY_DATABASE_DIR");
    expandPath(recoveryFile);
    recoveryFile.append("/" + opencog::config().get("PROXY_DATA_FILE"));

    if (fileExists(recoveryFile.c_str())) {
        recoveryFromPersistedData(recoveryFile);
    }

}

bool PVPSimulator::processNextMessage(MessagingSystem::Message *message)
{
    if (opencog::config().get_bool("GENERATE_GOLD_STANDARD")) {
        goldStdGen->writeMessage(*message, false);
    }
    if ( strcasestr( message->getPlainTextRepresentation(), "SUCCESS")) {
        string petId( message->getPlainTextRepresentation());
        logger().log(opencog::Logger::DEBUG, "SUCCESS: %s", petId.c_str());
        petId = petId.substr(petId.find_last_of(" ") + 1, petId.size());

        logger().log(opencog::Logger::INFO, "Pet \"%s\" was loaded with success.", petId.c_str());
        if (petsId.find(petId) == petsId.end())
            logger().log(opencog::Logger::ERROR, "There is no pet \"%s\" in PetsId list.", petId.c_str());
        else  petsId[petId] = true;
    } else {

        // used only to shutdown the simulator. Real PVP do not have something
        // like this
        if (message->getFrom() == opencog::config().get("SPAWNER_ID")) {
            if (std::string(message->getPlainTextRepresentation()) == "SAVE_AND_EXIT") {
                throw opencog::RuntimeException(TRACE_INFO, "PVPSim - Forcing a runtime exception.");
            }
        }

        parseXML(message->getPlainTextRepresentation());
    }
    return false;
}

bool PVPSimulator::sendMessage(Message &msg)
{
    if (opencog::config().get_bool("GENERATE_GOLD_STANDARD")) {
        goldStdGen->writeMessage(msg, true);
    }
    return EmbodimentCogServer::sendMessage(msg);
}

void PVPSimulator::timeTick()
{

    logger().log(opencog::Logger::DEBUG, "PVPSimulator::timeTick()");

    static_cast<SimulationConfig&>(opencog::config()).timeTick();
    worldSimulator->timeTick();
    for ( PetsPhysiologicalModelMap::iterator it = petsPhysiologicalModel.begin(); it != petsPhysiologicalModel.end(); it++ )
        it->second->timeTick();

    if ((tickCount % PET_SIGNAL_SENDING_INTERVAL) == 0) {
        sendPetSignals();
    }
    tickCount++;
}

void PVPSimulator::resetPhysiologicalModel(string petId)
{
    PetsPhysiologicalModelMap::iterator pet = petsPhysiologicalModel.find(petId);
    if (pet == petsPhysiologicalModel.end()) {
        logger().log(opencog::Logger::ERROR, "Could not reset physiological model of \"s\". Pet does not exist.", petId.c_str());
        return;
    }
    pet->second->reset();
}

void PVPSimulator::persistState() throw (opencog::IOException, std::bad_exception)
{

    // TODO: add some timestamp info to filename
    std::string path = opencog::config().get("PROXY_DATABASE_DIR");
    expandPath(path);

    if (!createDirectory(path.c_str())) {
        logger().log(opencog::Logger::ERROR, "PVPSimulator - Cannot create directory '%s'.",
                     path.c_str());
        return;
    }

    // TODO: Insert timestamp information into routerInfo file
    std::string filename = path + "/" + opencog::config().get("PROXY_DATA_FILE");
    remove(filename.c_str());

    std::ofstream pvpFile(filename.c_str());
    pvpFile.exceptions(std::ofstream::failbit | std::ofstream::badbit);

    try {

        PetsIdMap::const_iterator it;
        for (it = petsId.begin(); it != petsId.end(); it++) {

            // avoid saving information about an unavailable element
            if (isElementAvailable(it->first)) {
                // save data
                pvpFile << it->first << std::endl;
                pvpFile << tickCount << std::endl;
            }
        }
    } catch (std::ofstream::failure e) {
        pvpFile.close();
        throw opencog::IOException(TRACE_INFO, "PVPSimulator - Unable to save PVPSimulator information.");
    }

    pvpFile.close();
}

void PVPSimulator::recoveryFromPersistedData(const std::string& fileName)
{

    char line[256];
    std::ifstream fin(fileName.c_str());

    std::string id;
    int tick;

    while (!fin.eof()) {
        fin.getline(line, 256);

        // not a comentary or an empty line
        if (line[0] != '#' && line[0] != 0x00) {
            std::istringstream in ((std::string)line);

            in >> id;
            in >> tick;
            recoveryPetsId.insert(id);
        }
    }
    fin.close();
    remove(fileName.c_str());

    tickCount = tick;
}

void PVPSimulator::sendMessages()
{
    while (!messagesToSend.isQueueEmpty(PVP_ID) ) {
        logger().log(opencog::Logger::DEBUG, "PVPSimulator - Sending messages.");
        Message *msg = this->messagesToSend.pop(PVP_ID);

        // router available, send message. Otherwise just delete the message
        if (isElementAvailable(opencog::config().get("ROUTER_ID"))) {
            sendMessage(*msg);
        }

        delete(msg);
    }
}

std::string PVPSimulator::getCurrentTimestamp()
{
    static boost::posix_time::ptime epoch_time( PAIUtils::getSystemEpoch( ) );

    int seconds = static_cast<SimulationConfig&>(opencog::config()).getCurrentSimulationSeconds();
    boost::posix_time::ptime t = epoch_time + boost::posix_time::seconds(seconds);
    return to_iso_extended_string(t);
}

char *PVPSimulator::getNextToken(char *cursor, char *target)
{

    if (cursor[0] == '\0') {
        throw opencog::RuntimeException(TRACE_INFO, "Invalid action parameters. cursor[%s] target[%s]", cursor, target );
    }

    sscanf(cursor, "%s", target);

    while ((cursor[0] != ' ') && (cursor[0] != '\0')) {
        cursor++;
    }
    if (cursor[0] != '\0') {
        cursor++;
    }

    return cursor;
}

void PVPSimulator::addActionParameters(PetAction& action, std::string &xmlText, char * &cursor, ActionType &actionType, std::vector<ActionParamType> &typeList, bool required, unsigned int paramOffset)
{

    char paramValueStr[128];

    for (unsigned int i = 0; i < typeList.size(); i++) {

        if (cursor[0] == '\0') {
            if (required) {
                throw opencog::RuntimeException(TRACE_INFO, "Invalid action parameters");
            } else {
                return;
            }
        }

        unsigned int paramIndex = i + paramOffset;
        const std::string& name = actionType.getParamNames()[paramIndex];
        const ActionParamType& type = typeList[i];
        xmlText.append("<param name=\"");
        xmlText.append(name);
        xmlText.append("\" type=\"");
        xmlText.append(type.getName());
        switch (type.getCode()) {
        case BOOLEAN_CODE:
        case INT_CODE:
        case FLOAT_CODE:
        case STRING_CODE: {
            cursor = getNextToken(cursor, paramValueStr);
            xmlText.append("\" value=\"");
            xmlText.append(paramValueStr);
            xmlText.append("\"/>\n");
            ActionParameter p(name, type, paramValueStr);
            action.addParameter(p);
            break;
        }
        case VECTOR_CODE: {
            char x[128], y[128], z[128];
            cursor = getNextToken(cursor, x);
            cursor = getNextToken(cursor, y);
            cursor = getNextToken(cursor, z);
            xmlText.append("\">\n");
            xmlText.append("<vector x=\"");
            xmlText.append(x);
            xmlText.append("\" y=\"");
            xmlText.append(y);
            xmlText.append("\" z=\"");
            xmlText.append(z);
            xmlText.append("\"/>\n");
            xmlText.append("</param>\n");
            ActionParameter p(name, type, Vector(atof(x), atof(y), atof(z)));
            action.addParameter(p);
            break;
        }
        case ROTATION_CODE: {
            char pitch[128], roll[128], yaw[128];
            cursor = getNextToken(cursor, pitch);
            cursor = getNextToken(cursor, roll);
            cursor = getNextToken(cursor, yaw);
            xmlText.append("\">\n");
            xmlText.append("<rotation pitch=\"");
            xmlText.append(pitch);
            xmlText.append("\" roll=\"");
            xmlText.append(roll);
            xmlText.append("\" yaw=\"");
            xmlText.append(yaw);
            xmlText.append("\"/>\n");
            xmlText.append("</param>\n");
            ActionParameter p(name, type, Rotation(atof(pitch), atof(roll), atof(yaw)));
            action.addParameter(p);
            break;
        }
        case ENTITY_CODE: {
            char entityId[128], entityType[128];
            cursor = getNextToken(cursor, entityId);
            cursor = getNextToken(cursor, entityType);
            xmlText.append("\">\n");
            xmlText.append("<entity id=\"");
            xmlText.append(entityId);
            xmlText.append("\" type=\"");
            xmlText.append(entityType);
            xmlText.append("\"/>\n");
            xmlText.append("</param>\n");
            ActionParameter p(name, type, Entity(entityId, entityType));
            action.addParameter(p);
            break;
        }
        default: {
            throw opencog::RuntimeException(TRACE_INFO, "Invalid parameter code: %d", type.getCode());
        }
        }
    }
}

bool PVPSimulator::sendAgentAction( char *txt, const char *agentId, const char* agentType )
{

    bool result = true;

    if ( avatarsId.find(agentId) == avatarsId.end() && petsId.find(agentId) == petsId.end( ) ) {
        logger().log(opencog::Logger::ERROR, "ERROR - PVPSimulator - There is not agent with Id: %s", agentId);
        return false;
    }

    logger().log(opencog::Logger::INFO, "Sending agent action: %s to id: %s", txt, agentId);
    std::string xmlText;

    pthread_mutex_lock(&currentTimeLock);

    std::string timestamp = getCurrentTimestamp();

    char *cursor = txt;
    char actionName[128];

    try {
        cursor = getNextToken(cursor, actionName);

        xmlText.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        xmlText.append("<pet:petaverse-msg xmlns:pet=\"http://proxy.esheepco.com/brain\">\n");
        xmlText.append("<agent-signal agent-id=\"");
        xmlText.append(agentId);
        xmlText.append("\" agent-type=\"");
        xmlText.append(agentType);
        xmlText.append("\" name=\"");
        xmlText.append(actionName);
        xmlText.append("\" timestamp=\"");
        xmlText.append(timestamp);
        xmlText.append("\">\n");

        ActionType actionType = ActionType::getFromName(actionName);
        PetAction action(actionType);
        std::vector<ActionParamType> requiredTypeList = actionType.getMandatoryParamTypes();
        std::vector<ActionParamType> optionalTypeList = actionType.getOptionalParamTypes();

        logger().log(opencog::Logger::DEBUG, "parsing required parameters (cursor = %s)", cursor);
        addActionParameters(action, xmlText, cursor, actionType, requiredTypeList, true, 0);
        logger().log(opencog::Logger::DEBUG, "parsing optional parameters (cursor = %s)", cursor);
        addActionParameters(action, xmlText, cursor, actionType, optionalTypeList, false, requiredTypeList.size());

        xmlText.append("</agent-signal>\n");
        xmlText.append("</pet:petaverse-msg>");

        worldSimulator->executeAction(agentId, action);

        logger().log(opencog::Logger::FINE, "xmlText = \n%s\n", xmlText.c_str());

        // all pets are going to receive agent actions message of all agents
        for ( PetsIdMap::iterator itPet = petsId.begin(); itPet != petsId.end(); itPet++ ) {
            if (!isElementAvailable(itPet->first) || itPet->first == agentId ) {
                // avoid creating agent action messages if the destiny (petId) isn't available
                // avoid sending an agent-signal to himself
                continue;
            } // if

            StringMessage *message = new StringMessage(getID(), itPet->first, xmlText);
            messagesToSend.push(PVP_ID, message);
        }
    } catch (opencog::RuntimeException &e) {
        logger().log(opencog::Logger::ERROR, "Could not create action message to send to agent '%s'. The exception '%s' occured.", agentId, e.getMessage() );
        result = false;
    }

    pthread_mutex_unlock(&currentTimeLock);

    return result;
}

bool PVPSimulator::sendOwnerAction(char *txt)
{
    logger().log(opencog::Logger::DEBUG, "DEBUG - PVPSimulator - Action:  %s to Owner: %s", txt, DEFAULT_OWNER_ID);
    return sendAgentAction(txt, DEFAULT_OWNER_ID, "avatar" );
}

void PVPSimulator::sendPredavese(const char *txt, string petId)
{

    if (!isElementAvailable(petId)) {
        printf("PETUNAVAILABLE %s\n", petId.c_str()); fflush(stdout);
        return;
    }

    if (!isElementAvailable(opencog::config().get("ROUTER_ID"))) {
        printf("ROUTERUNAVAILABLE\n"); fflush(stdout);
        return;
    }


    logger().log(opencog::Logger::INFO, "Sending predavese text: %s to %s)", txt, petId.c_str());

    if ( petsId[petId] == false ) {
        logger().log(opencog::Logger::ERROR, "The succes load message of pet: %s did not receive  yet", petId.c_str());
        return;
    }
    std::string xmlText;
    string ownerId = ownership[petId];

    pthread_mutex_lock(&currentTimeLock);

    std::string timestamp = getCurrentTimestamp();

    xmlText.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xmlText.append("<pet:petaverse-msg xmlns:pet=\"http://proxy.esheepco.com/brain\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://proxy.esheepco.com/brain BrainProxyAxon.xsd\">\n");
    xmlText.append("<instruction pet-id=\"");
    xmlText.append(petId);
    xmlText.append("\" avatar-id=\"");
    xmlText.append(ownerId);
    xmlText.append("\" timestamp=\"");
    xmlText.append(timestamp);
    xmlText.append("\">\n");
    xmlText.append("<value>");
    xmlText.append(txt);
    xmlText.append("</value>\n");
    xmlText.append("</instruction>\n");
    xmlText.append("</pet:petaverse-msg>");

    logger().log(opencog::Logger::FINE, "xmlText = \n%s\n", xmlText.c_str());
    StringMessage *message = new StringMessage(getID(), petId, xmlText);

    messagesToSend.push(PVP_ID, message);
    pthread_mutex_unlock(&currentTimeLock);

}

void PVPSimulator::sendPetSignals()
{

    logger().log(opencog::Logger::INFO, "Sending pet signal (pet physiological states):");

    char hunger[16];
    char thirst[16];
    char peeUrgency[16];
    char pooUrgency[16];
    char fitness[16];
    char energy[16];

    pthread_mutex_lock(&currentTimeLock);

    for ( PetsPhysiologicalModelMap::iterator it = petsPhysiologicalModel.begin(); it != petsPhysiologicalModel.end(); it++) {

        string petId = it->first;
        if ((petsId[petId] == false)) {
            logger().log(opencog::Logger::WARN, "Pet \"%s\" did not loaded yet.\n", petId.c_str());
            continue;
        }
        // avoid creating messages if the destiny (petId) isn't available
        if  (!isElementAvailable(petId)) {
            logger().log(opencog::Logger::WARN, "Unavailable element for Pet \"%s\".\n", petId.c_str());
            continue;
        }

        PhysiologicalModel *physiologicalModel = it->second;

        sprintf(hunger, "%f", physiologicalModel->getHunger());
        sprintf(thirst, "%f", physiologicalModel->getThirst());
        sprintf(peeUrgency, "%f", physiologicalModel->getPeeUrgency());
        sprintf(pooUrgency, "%f", physiologicalModel->getPooUrgency());
        sprintf(fitness, "%f", physiologicalModel->getScaledFitness());
        sprintf(energy, "%f", physiologicalModel->getScaledEnergy());

        logger().log(opencog::Logger::INFO, "%s::hungerLevel = %s", petId.c_str(), hunger);
        logger().log(opencog::Logger::INFO, "%s::thirstLevel = %s", petId.c_str(), thirst);
        logger().log(opencog::Logger::INFO, "%s::peeUrgencyLevel = %s", petId.c_str(), peeUrgency);
        logger().log(opencog::Logger::INFO, "%s::pooUrgencyLevel = %s", petId.c_str(), pooUrgency);
        logger().log(opencog::Logger::INFO, "%s::fitnessLevel = %s", petId.c_str(), fitness);
        logger().log(opencog::Logger::INFO, "%s::energyLevel = %s", petId.c_str(), energy);

        printf("PETSTATUS %s hungerLevel %s\n", petId.c_str(), hunger); fflush(stdout);
        printf("PETSTATUS %s thirstLevel %s\n", petId.c_str(), thirst); fflush(stdout);
        printf("PETSTATUS %s peeUrgencyLevel %s\n", petId.c_str(), peeUrgency); fflush(stdout);
        printf("PETSTATUS %s pooUrgencyLevel %s\n", petId.c_str(), pooUrgency); fflush(stdout);
        printf("PETSTATUS %s fitnessLevel %s\n", petId.c_str(), fitness); fflush(stdout);
        printf("PETSTATUS %s energyLevel %s\n", petId.c_str(), energy); fflush(stdout);

        std::string xmlText;
        std::string timestamp = getCurrentTimestamp();

        xmlText.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        xmlText.append("<pet:petaverse-msg xmlns:pet=\"http://proxy.esheepco.com/brain\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://proxy.esheepco.com/brain BrainProxyAxon.xsd\">\n");

        xmlText.append("<pet-signal pet-id=\"");
        xmlText.append(petId);
        xmlText.append("\" name=\"hunger\" timestamp=\"");
        xmlText.append(timestamp);
        xmlText.append("\">\n");
        xmlText.append("<param name=\"level\" type=\"float\" value=\"");
        xmlText.append(hunger);
        xmlText.append("\" />\n");
        xmlText.append("</pet-signal>\n");

        xmlText.append("<pet-signal pet-id=\"");
        xmlText.append(petId);
        xmlText.append("\" name=\"thirst\" timestamp=\"");
        xmlText.append(timestamp);
        xmlText.append("\">\n");
        xmlText.append("<param name=\"level\" type=\"float\" value=\"");
        xmlText.append(thirst);
        xmlText.append("\" />\n");
        xmlText.append("</pet-signal>\n");

        xmlText.append("<pet-signal pet-id=\"");
        xmlText.append(petId);
        xmlText.append("\" name=\"pee_urgency\" timestamp=\"");
        xmlText.append(timestamp);
        xmlText.append("\">\n");
        xmlText.append("<param name=\"level\" type=\"float\" value=\"");
        xmlText.append(peeUrgency);
        xmlText.append("\" />\n");
        xmlText.append("</pet-signal>\n");

        xmlText.append("<pet-signal pet-id=\"");
        xmlText.append(petId);
        xmlText.append("\" name=\"poo_urgency\" timestamp=\"");
        xmlText.append(timestamp);
        xmlText.append("\">\n");
        xmlText.append("<param name=\"level\" type=\"float\" value=\"");
        xmlText.append(pooUrgency);
        xmlText.append("\" />\n");
        xmlText.append("</pet-signal>\n");

        xmlText.append("<pet-signal pet-id=\"");
        xmlText.append(petId);
        xmlText.append("\" name=\"fitness\" timestamp=\"");
        xmlText.append(timestamp);
        xmlText.append("\">\n");
        xmlText.append("<param name=\"level\" type=\"float\" value=\"");
        xmlText.append(fitness);
        xmlText.append("\" />\n");
        xmlText.append("</pet-signal>\n");

        xmlText.append("<pet-signal pet-id=\"");
        xmlText.append(petId);
        xmlText.append("\" name=\"energy\" timestamp=\"");
        xmlText.append(timestamp);
        xmlText.append("\">\n");
        xmlText.append("<param name=\"level\" type=\"float\" value=\"");
        xmlText.append(energy);
        xmlText.append("\" />\n");
        xmlText.append("</pet-signal>\n");

        xmlText.append("</pet:petaverse-msg>\n");

        logger().log(opencog::Logger::FINE, "xmlText = \n%s\n", xmlText.c_str());
        StringMessage *message = new StringMessage(getID(), petId, xmlText);

        messagesToSend.push(PVP_ID, message);

        Message *tick_message = Message::factory(getID(), petId, Message::TICK, getID());
        messagesToSend.push(PVP_ID, tick_message);

    }

    pthread_mutex_unlock(&currentTimeLock);

}

void PVPSimulator::sendActionStatusPetSignal(const std::string& planId, const std::string& petId, const PerceptionActionInterface::PetAction& action, bool success)
{
    std::string xmlText;

    if ( petsId[petId] == false ) {
        logger().log(opencog::Logger::WARN, "Pet \"%s\" did not loaded yet.\n", petId.c_str() );
        return;
    }

    pthread_mutex_lock(&currentTimeLock);

    std::string timestamp = getCurrentTimestamp();

    xmlText.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xmlText.append("<pet:petaverse-msg xmlns:pet=\"http://proxy.esheepco.com/brain\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://proxy.esheepco.com/brain BrainProxyAxon.xsd\">\n");

    xmlText.append("<pet-signal pet-id=\"");
    xmlText.append(petId);
    xmlText.append("\" timestamp=\"");
    xmlText.append(timestamp);
    xmlText.append("\" action-plan-id=\"");
    xmlText.append(planId);
    xmlText.append("\" sequence=\"");
    xmlText.append(opencog::toString(action.getSequence()));
    xmlText.append("\" name=\"");
    xmlText.append(action.getName());
    xmlText.append("\" status=\"");
    xmlText.append(success ? "done" : "error");
    xmlText.append("\"/>\n");

    xmlText.append("</pet:petaverse-msg>\n");

    logger().log(opencog::Logger::FINE, "xmlText = \n%s\n", xmlText.c_str());
    StringMessage *message = new StringMessage(getID(), petId, xmlText);

    messagesToSend.push(PVP_ID, message);
    pthread_mutex_unlock(&currentTimeLock);

    if ( success ) {
        // send to all loaded pets a message notifying about a pet action
        std::stringstream parameters;
        parameters << action.getName( );
        const std::list<ActionParameter>& actionParameters = action.getParameters( );
        std::list<ActionParameter>::const_iterator it;
        for ( it = actionParameters.begin( ); it != actionParameters.end( ); ++it ) {
            parameters << " ";
            parameters << it->stringRepresentation( );
        } // if
        char* message = new char[parameters.str( ).length( )];
        strcpy( message, parameters.str( ).c_str( ) );
        sendAgentAction( message, petId.c_str( ), "pet" );
    } // if
}

void PVPSimulator::sendActionStatusPetSignal(const std::string& planId, const std::string& petId, bool success)
{
    std::string xmlText;

    if ( petsId[petId] == false ) {
        logger().log(opencog::Logger::WARN, "Pet \"%s\" did not loaded yet.\n", petId.c_str() );
        return;
    }

    pthread_mutex_lock(&currentTimeLock);

    std::string timestamp = getCurrentTimestamp();

    xmlText.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xmlText.append("<pet:petaverse-msg xmlns:pet=\"http://proxy.esheepco.com/brain\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://proxy.esheepco.com/brain BrainProxyAxon.xsd\">\n");

    xmlText.append("<pet-signal pet-id=\"");
    xmlText.append(petId);
    xmlText.append("\" timestamp=\"");
    xmlText.append(timestamp);
    xmlText.append("\" action-plan-id=\"");
    xmlText.append(planId);
    xmlText.append("\" status=\"");
    xmlText.append(success ? "done" : "error");
    xmlText.append("\"/>\n");

    xmlText.append("</pet:petaverse-msg>\n");

    logger().log(opencog::Logger::FINE, "xmlText = \n%s\n", xmlText.c_str());
    StringMessage *message = new StringMessage(getID(), petId, xmlText);

    messagesToSend.push(PVP_ID, message);
    pthread_mutex_unlock(&currentTimeLock);

}

//<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
//<pet:action-plan xmlns:pet="http://proxy.esheepco.com/brain" id="7" pet-id="1">
//<action name="heel" sequence="1"/>
//</pet:action-plan>

void PVPSimulator::parseActionPlan(XERCES_CPP_NAMESPACE::DOMElement *actionPlan)
{

    XMLCh tag[PAIUtils::MAX_TAG_LENGTH + 1];

    // get pet id
    XERCES_CPP_NAMESPACE::XMLString::transcode(ENTITY_ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* entityId = XERCES_CPP_NAMESPACE::XMLString::transcode(actionPlan->getAttribute(tag));
    std::string petId(entityId);

    // received a new action plan and previous action plan hasn't finished its
    // execution - cancel previous action plan
    if (!petActionsList[petId].empty()) {
        cancelActionPlan(petId);
    }

    XERCES_CPP_NAMESPACE::XMLString::transcode(ID_ATTRIBUTE, tag, PAIUtils::MAX_TAG_LENGTH);
    char* planId = XERCES_CPP_NAMESPACE::XMLString::transcode(actionPlan->getAttribute(tag));
    currentPlanId[petId] = string(planId);

    XERCES_CPP_NAMESPACE::XMLString::transcode(ACTION_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    XERCES_CPP_NAMESPACE::DOMNodeList *list = actionPlan->getElementsByTagName(tag);

    for (unsigned int i = 0; i < list->getLength(); i++) {
        PetAction * action = PetAction::factory((XERCES_CPP_NAMESPACE::DOMElement*) list->item(i));

        // remember list with FIFO policy
        petActionsList[petId].push_front(action);
    }

    XERCES_CPP_NAMESPACE::XMLString::release(&planId);
    processPetActions(petId);
}

void PVPSimulator::cancelActionPlan(const std::string& petId)
{

    PetActionsList& actionsList = petActionsList[petId];
    while (!actionsList.empty()) {
        PetAction * action = actionsList.front();
        actionsList.pop_front();
        delete action;
    }
    sendActionStatusPetSignal(currentPlanId[petId], petId, false);
    currentPlanId[petId].erase();

    std::set<unsigned long> ticketsToRemove;

    // clear previous scheduled actions
    for ( ActionTicketToPetIdMap::reverse_iterator it = ticketToPetId.rbegin(); it != ticketToPetId.rend(); it++ ) {
        if ( it->second == petId ) {
            ticketsToRemove.insert(it->first);
        }
    }

    for (std::set<unsigned long>::iterator itr = ticketsToRemove.begin();
            itr != ticketsToRemove.end(); itr++) {
        ticketToActionMap.erase(*itr);
        ticketToPlanIdMap.erase(*itr);
        ticketToPetId.erase(*itr);
    }
}

void PVPSimulator::processPetActions(const std::string& petId)
{

    PetActionsList& actionsList = petActionsList[petId];
    if (actionsList.empty()) {
        // processed all actions just return
        return;
    }

    string currPlanId = currentPlanId[petId];
    PhysiologicalModel *physiologicalModel = petsPhysiologicalModel[petId];

    // control if should way an action to finish its execution to start a new
    // one
    bool waitAction = false;

    while (!actionsList.empty()) {

        // remember list with FIFO policy
        PetAction * action = actionsList.back();

        unsigned long actionTicket = worldSimulator->executeAction(petId, *action);
        printf("PETACTION %lu %s %s\n", actionTicket, petId.c_str(), action->stringRepresentation().c_str());
        fflush(stdout);

        if (actionTicket == 0) {
            waitAction = false;
            physiologicalModel->processCommand(*action);
            sendActionStatusPetSignal(currPlanId, petId, *action, true);
            printf("PETACTIONSTATUS last OK\n"); fflush(stdout);

        } else if (actionTicket == ULONG_MAX) {
            waitAction = false;
            sendActionStatusPetSignal(currPlanId, petId, *action, false);
            printf("PETACTIONSTATUS last FAILED\n"); fflush(stdout);

        } else if (actionTicket == TEST_TICKET) {
            waitAction = false;
            ticketToActionMap[actionTicket] = *action;
            ticketToPlanIdMap[actionTicket] = currPlanId;
            ticketToPetId[actionTicket] = petId;

        } else {
            waitAction = true;
            ticketToActionMap[actionTicket] = *action;
            ticketToPlanIdMap[actionTicket] = currPlanId;
            ticketToPetId[actionTicket] = petId;
        }

        // list with FIFO policy
        actionsList.pop_back();
        delete action;

        if (waitAction) {
            return;
        }
    }
}

void PVPSimulator::parseDOMDocument(XERCES_CPP_NAMESPACE::DOMDocument *document)
{

    logger().log(opencog::Logger::DEBUG, "parseDOMDocument");
    XMLCh tag[PAIUtils::MAX_TAG_LENGTH + 1];
    XERCES_CPP_NAMESPACE::DOMNodeList *list;

    XERCES_CPP_NAMESPACE::XMLString::transcode(ACTION_PLAN_ELEMENT, tag, PAIUtils::MAX_TAG_LENGTH);
    list = document->getElementsByTagName(tag);
    logger().log(opencog::Logger::DEBUG, "action plan count = %d", list->getLength());

    for (unsigned int i = 0; i < list->getLength(); i++) {
        parseActionPlan((XERCES_CPP_NAMESPACE::DOMElement *)list->item(i));
    }
}

void PVPSimulator::publicParseXML(const std::string& xmlText)
{
    parseXML(xmlText);
}

bool PVPSimulator::parseXML(const std::string& xmlText)
{

    logger().log(opencog::Logger::DEBUG, "PVPSimulator::parseXML()");
    if (logger().getLevel() >= opencog::Logger::FINE)
        logger().log(opencog::Logger::FINE, "Processing XML:\n%s", xmlText.c_str());

    static const char *bufID = strdup("xml message");
    const XMLByte *xmlBuf = reinterpret_cast<const XMLByte *>(xmlText.c_str());
    logger().log(opencog::Logger::FINE, "PVPSimulator::parseXML() - building input buffer");
    XERCES_CPP_NAMESPACE::MemBufInputSource *memBufIS = new XERCES_CPP_NAMESPACE::MemBufInputSource(xmlBuf, xmlText.size(), bufID);

    logger().log(opencog::Logger::FINE, "PVPSimulator::parseXML() - building PetaverseDOMParser");
    PetaverseDOMParser *parser = new PetaverseDOMParser();
    PetaverseErrorHandler errorHandler;
    parser->setErrorHandler(&errorHandler);

    bool success = true;

    try {
        logger().log(opencog::Logger::DEBUG, "PVPSimulator::parseXML() - parsing");
        parser->parse(*memBufIS);
    } catch (const XERCES_CPP_NAMESPACE::XMLException& toCatch) {
        char* message = XERCES_CPP_NAMESPACE::XMLString::transcode(toCatch.getMessage());
        logger().log(opencog::Logger::WARN, "XML Exception: %s\n", message);
        XERCES_CPP_NAMESPACE::XMLString::release(&message);
        success = false;
    } catch (const XERCES_CPP_NAMESPACE::DOMException& toCatch) {
        char* message = XERCES_CPP_NAMESPACE::XMLString::transcode(toCatch.msg);
        logger().log(opencog::Logger::ERROR, "DOM Exception: %s\n", message);
        XERCES_CPP_NAMESPACE::XMLString::release(&message);
        success = false;
    } catch (...) {
        logger().log(opencog::Logger::ERROR, "Unexpected XML Parse Exception\n");
        success = false;
    }

    if (success) {
        logger().log(opencog::Logger::DEBUG, "Parsing suceed");
    } else {
        logger().log(opencog::Logger::DEBUG, "Parsing failed");
    }

    XERCES_CPP_NAMESPACE::DOMDocument *document = NULL;
    if (success) {
        if (parser->getErrorCount() == 0) {
            logger().log(opencog::Logger::FINE, "Building DOMDocument");
            document = parser->adoptDocument();
            parseDOMDocument(document);
            //document->release();
        } else {
            logger().log(opencog::Logger::ERROR, "Got %d errors buildiong DOM document\n", parser->getErrorCount());
            success = false;
        }
    }

    logger().log(opencog::Logger::FINE, "PVPSimulator::parseXML() - cleaning up memory");

    delete memBufIS;
    delete parser;
//   delete document;

//    delete bufID;

    return success;
}

void PVPSimulator::setWorldSimulator(WorldSimulator *worldSimulator)
{
    delete(this->worldSimulator);
    this->worldSimulator = worldSimulator;
}

void PVPSimulator::setPhysiologicalModel(PhysiologicalModel *physiologicalModel)
{
//    delete(this->physiologicalModel);
//    this->physiologicalModel = physiologicalModel;
}

void PVPSimulator::mapInfo(std::vector<ObjMapInfo>& objects)
{
    pthread_mutex_lock(&currentTimeLock);
    logger().log(opencog::Logger::INFO, "Building and sending map-info message:");

    // Build a map-info message with a blip for that object and sends it to PB Router
    std::string xmlText;
    std::string timestamp = getCurrentTimestamp();

    xmlText.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xmlText.append("<pet:petaverse-msg xmlns:pet=\"http://proxy.esheepco.com/brain\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://proxy.esheepco.com/brain BrainProxyAxon.xsd\">\n");
    xmlText.append("<map-info global-position-x=\"-64\" global-position-y=\"-64\" global-position-offset=\"128\">\n"); // TODO: get the global coordinates from AGISimSim somehow?

    foreach(ObjMapInfo obj, objects) {
        xmlText.append("<blip timestamp=\"");
        xmlText.append(timestamp);
        if (obj.removed) {
            xmlText.append("\" remove=\"true\" detector=\"true\">\n");
        } else {
            xmlText.append("\" width=\"");
            xmlText.append(opencog::toString(obj.width));
            xmlText.append("\" length=\"");
            xmlText.append(opencog::toString(obj.length));
            xmlText.append("\" height=\"");
            xmlText.append(opencog::toString(obj.height));
            xmlText.append("\" edible=\"");
            xmlText.append((obj.edible) ? "true" : "false");
            xmlText.append("\" drinkable=\"");
            xmlText.append((obj.drinkable) ? "true" : "false");
            xmlText.append("\" petHome=\"");
            xmlText.append((obj.petHome) ? "true" : "false");
            xmlText.append("\" foodBowl=\"");
            xmlText.append((obj.foodBowl) ? "true" : "false");
            xmlText.append("\" waterBowl=\"");
            xmlText.append((obj.waterBowl) ? "true" : "false");
            xmlText.append("\" detector=\"true\">\n");
        }

        // TODO: Find a way to identify the pet's name and owner
        //bool isPet = !strcasecmp(obj.name.c_str(),PET_ID);
        xmlText.append("<entity name=\"");
        xmlText.append(obj.name);
        xmlText.append("\" id=\"");
        xmlText.append(obj.name);
        xmlText.append("\" type=\"");
        if (obj.type != "pet" &&
                obj.type != "avatar" &&
                obj.type != "object" &&
                obj.type != "accessory" &&
                obj.type != "structure" &&
                obj.type != "unknown") {
            xmlText.append("unknown");
        } else {
            xmlText.append(obj.type);
        }
        if (obj.type == "pet") {
            xmlText.append("\" owner-id=\"");
            xmlText.append( ownership[obj.name] );
            xmlText.append("\" owner-name=\"");
            xmlText.append( ownership[obj.name] );
        }
        xmlText.append("\"/>\n");

        if (obj.removed) {
            // Add fake position, rotaion and velocity elements, since XSD requires them...
            xmlText.append("<position x=\"0\" y=\"0\" z=\"0\"/>\n");
            xmlText.append("<rotation pitch=\"0\" roll=\"0\" yaw=\"0\"/>\n");
            xmlText.append("<velocity x=\"0\" y=\"0\" z=\"0\"/>\n");
        } else {
            xmlText.append("<position x=\"");
            xmlText.append(opencog::toString(obj.posX));
            xmlText.append("\" y=\"");
            xmlText.append(opencog::toString(obj.posZ)); // AGISimSim z => SL y
            xmlText.append("\" z=\"");
            xmlText.append(opencog::toString(obj.posY)); // AGISimSim y => SL z
            xmlText.append("\"/>\n");
            xmlText.append("<rotation pitch=\"");
            xmlText.append(opencog::toString(obj.rotX));
            xmlText.append("\" roll=\"");
            xmlText.append(opencog::toString(obj.rotZ)); // AGISimSim z => SL y
            xmlText.append("\" yaw=\"");
            xmlText.append(opencog::toString(obj.rotY)); // AGISimSim y => SL z
            xmlText.append("\"/>\n");
            // Sets the velocity property according with the variation of position over the latest period of time
            // TODO: For now, do not divide the difference of position by the time, since this is not used at the PB side
            // (i.e., in PB, we only need to know if the object has been moving in the recent past or not)
            Vector velocity(0, 0, 0);
            Vector newPos(obj.posX, obj.posZ, obj.posY);
            ObjTimestampMap::iterator ts_it = objTimestamp.find(obj.name);
            if (ts_it != objTimestamp.end()) {
                const std::string& previousTimestamp = ts_it->second;
                ObjPositionMap::iterator pos_it = objPosition.find(obj.name);
                Vector pos = pos_it->second;
                ObjPositionMap::iterator prevPos_it = objPreviousPosition.find(obj.name);
                Vector previousPos = prevPos_it->second;
                if (newPos.x != pos.x || newPos.y != pos.y || newPos.z != pos.z) {
                    velocity = Vector(newPos.x - pos.x, newPos.y - pos.y, newPos.z - pos.z);
                } else if (previousPos.x != pos.x || previousPos.y != pos.y || previousPos.z != pos.z) {
                    velocity = Vector(pos.x - previousPos.x, pos.y - previousPos.y, pos.z - previousPos.z);
                }
                if (previousTimestamp != timestamp) {
                    objTimestamp[obj.name] = timestamp;
                    objPosition[obj.name] = newPos;
                    objPreviousPosition[obj.name] = pos;
                }
            } else {
                objTimestamp[obj.name] = timestamp;
                objPosition[obj.name] = newPos;
                objPreviousPosition[obj.name] = newPos;
            }
            xmlText.append("<velocity x=\"");
            xmlText.append(opencog::toString(velocity.x));
            xmlText.append("\" y=\"");
            xmlText.append(opencog::toString(velocity.y));
            xmlText.append("\" z=\"");
            xmlText.append(opencog::toString(velocity.z));
            xmlText.append("\"/>\n");
        }
        xmlText.append("</blip>\n");
    }
    xmlText.append("</map-info>\n");
    xmlText.append("</pet:petaverse-msg>\n");

    logger().log(opencog::Logger::FINE, "xmlText = \n%s\n", xmlText.c_str());

    for ( PetsIdMap::iterator itPetId = petsId.begin(); itPetId != petsId.end(); itPetId++ ) {

        if (!petsId[itPetId->first]) {
            logger().log(opencog::Logger::WARN, "Pet \"%s\" did not loaded yet.\n", itPetId->first.c_str());
            continue;
        }
        // avoid creating map-info messages if the destiny isn't available
        if  (!isElementAvailable(itPetId->first)) {
            logger().log(opencog::Logger::WARN, "Unavailable element for Pet \"%s\".\n", itPetId->first.c_str());
            continue;
        }

        StringMessage *message = new StringMessage(getID(), itPetId->first, xmlText);
        messagesToSend.push(PVP_ID, message);
    }
    pthread_mutex_unlock(&currentTimeLock);

}

void PVPSimulator::actionStatus(unsigned long actionTicket, bool success)
{
    logger().log(opencog::Logger::INFO,
                 "Received action status for action with ticket = %lu success = %s",
                 actionTicket, success ? "true" : "false");

    ActionTicketToActionMap::iterator it = ticketToActionMap.find(actionTicket);
    if (it != ticketToActionMap.end()) {

        const string& planId = ticketToPlanIdMap[actionTicket];
        ActionTicketToPetIdMap::iterator itPetId = ticketToPetId.find(actionTicket);
        if ( itPetId == ticketToPetId.end() ) {
            logger().log(opencog::Logger::DEBUG, "Not found pet id to ticket '%lu' (plan id '%s')", actionTicket, planId.c_str());
            return;
        }
        const string petId = itPetId->second;

        PhysiologicalModel *physiologicalModel = petsPhysiologicalModel[petId];
        sendActionStatusPetSignal(planId, petId, it->second, success);

        if (success) {
            physiologicalModel->processCommand(it->second);
            printf("PETACTIONSTATUS %lu OK\n", actionTicket); fflush(stdout);

        } else {
            printf("PETACTIONSTATUS %lu FAILED\n", actionTicket); fflush(stdout);
        }

        ticketToActionMap.erase(actionTicket);
        ticketToPlanIdMap.erase(actionTicket);
        ticketToPetId.erase(actionTicket);

        // process remaining messages, if any
        processPetActions(petId);

    } else {
        logger().log(opencog::Logger::WARN,
                     "Could not find any action associated with the ticket = %lu",
                     actionTicket);
    }
}

void PVPSimulator::errorNotification(const std::string& errorMsg)
{
    logger().log(opencog::Logger::ERROR, "WorldSimulator error: %s\n", errorMsg.c_str());
}

bool PVPSimulator::connectToSimWorld()
{
    string createdOwnerId = createAvatar(DEFAULT_OWNER_ID, 30, 50);
    string createdPetId = createPet(DEFAULT_PET_ID, createdOwnerId, 50, 45);
    if ( createdPetId == "" ) {
        logger().log(opencog::Logger::ERROR, "Could not create Pet: '%s'", DEFAULT_PET_ID);
        return false;
    }
    return true;
}


std::string PVPSimulator::createPet(const std::string& petId, const std::string& ownerId, float x, float y)
{
    if (avatarsId.find(ownerId) == avatarsId.end()) {
        logger().log(opencog::Logger::ERROR, "There is not avatar: '%s' created.",  ownerId.c_str());
        return "";
    }

    const std::string& createdPetId = worldSimulator->createAgent(petId, "pet", x, y, true);
    logger().log(opencog::Logger::INFO, "Pet created with id = '%s'.", createdPetId.c_str());
    if (createdPetId != petId) {
        logger().log(opencog::Logger::WARN, "Could not create Pet with name = %s", petId.c_str());
    }
    petsId[createdPetId] = false;
    ownership[createdPetId] = ownerId;
    petsPhysiologicalModel[createdPetId] = new PhysiologicalModel();

    printf("PETCREATED %s\n", createdPetId.c_str()); fflush(stdout);
    // recovey data. PVP went down but pet is still executing fine. No need to
    // reload it
    if (recoveryPetsId.find(createdPetId) != recoveryPetsId.end()) {
        petsId[createdPetId] = true;

    } else {
        loadPet( createdPetId );
    }

    return createdPetId;
}

std::string PVPSimulator::createAvatar(const std::string& avatarId, float x, float y)
{

    std::string createdAvatarId = worldSimulator->createAgent(avatarId, "avatar", x, y, true);

    logger().log(opencog::Logger::INFO, "Avatar created with id = '%s'.", createdAvatarId.c_str());
    if (avatarId != createdAvatarId) {
        logger().log(opencog::Logger::WARN, "Could not create avatar with id: '%s'.", avatarId.c_str());
    }
    avatarsId.insert(createdAvatarId);

    printf("AVATARCREATED %s\n", createdAvatarId.c_str()); fflush(stdout);

    return createdAvatarId;
}

bool PVPSimulator::loadPet(const std::string& petId)
{
    if ( petsId.find(petId) == petsId.end() )  {
        logger().log(opencog::Logger::ERROR, "Could not load pet: '%s'. It was not created.", petId.c_str());
        return false;
    }

    std::string msgCmd = "LOAD_PET ";
    msgCmd += petId;
    MessagingSystem::StringMessage msg(opencog::config().get("PROXY_ID"),
                                       opencog::config().get("SPAWNER_ID"),
                                       msgCmd);
    sendMessage(msg);
    return  true;
}
