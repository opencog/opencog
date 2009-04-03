/*
 * opencog/embodiment/Control/MessagingSystem/Spawner.cc
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

#ifndef WIN32
#include <unistd.h>
#endif

#include <LoggerFactory.h>
#include "Spawner.h"
#include "StringMessage.h"
#include <iostream>
#include <sstream>

using namespace MessagingSystem;
using namespace opencog;

BaseServer* Spawner::createInstance()
{
    return new Spawner;
}

Spawner::~Spawner()
{
}

Spawner::Spawner()
{
}

void Spawner::init(const Control::SystemParameters &params,
                   const std::string &id,
                   const std::string &ip,
                   int port) throw (opencog::InvalidParamException, std::bad_exception)
{

    setNetworkElement(new NetworkElement(params, id, ip, port));

    minOpcPort = atoi(getParameters().get("MIN_OPC_PORT").c_str());
    maxOpcPort = atoi(getParameters().get("MAX_OPC_PORT").c_str());
    int maxNumberOfPets = maxOpcPort - minOpcPort;

    if (minOpcPort <= 0 || maxOpcPort <= 0 || maxNumberOfPets < 0) {
        throw opencog::InvalidParamException(TRACE_INFO,
                                             "Spawner - Invalid port range defined by (Min '%d', Max '%d').",
                                             minOpcPort, maxOpcPort);
    }
}

extern unsigned sleep(unsigned useconds);

int Spawner::allocateOpcPort(const std::string& petId)
{
    std::map<std::string, int>::const_iterator itr = petId2PortMap.find(petId);
    if (itr != petId2PortMap.end()) {
        // there is a port already allocated to this petId. Returns it.
        return itr->second;
    }
    for (int port = minOpcPort; port <= maxOpcPort; port++) {
        if (port2PetIdMap.find(port) == port2PetIdMap.end()) {
            petId2PortMap[petId] = port;
            port2PetIdMap[port] = petId;
            return port;
        }
    }
    return -1;
}

void Spawner::releaseOpcPort(const std::string& petId)
{
    std::map<std::string, int>::const_iterator itr = petId2PortMap.find(petId);
    if (itr != petId2PortMap.end()) {
        port2PetIdMap.erase(itr->second);
        petId2PortMap.erase(petId);
    }
}

bool Spawner::processNextMessage(Message *message)
{

    std::string cmdLine = message->getPlainTextRepresentation();
    std::string command;
    std::queue<std::string> args;

    NetworkElement::parseCommandLine(cmdLine, command, args);
    logger().log(opencog::Logger::INFO, "Spawner - line(%s) command(%s) # of parsed arguments(%d).", cmdLine.c_str( ), command.c_str( ), args.size( ) );

    if (command == "LOAD_AGENT") {
        std::string agentID = args.front();
        args.pop( );
        std::string ownerID = args.front();
        args.pop( );
        std::string agentType = args.front();
        args.pop( );
        std::string agentTraits = args.front();

        logger().log(opencog::Logger::INFO, "Spawner - agentId(%s) ownerId(%s) agentType(%s) agentTraits(%s).", agentID.c_str( ), ownerID.c_str( ), agentType.c_str( ), agentTraits.c_str( ) );

        // TODO: Find a way to figure out that OPC is already running.
        // The call to isElementAvailable method does not work because it only knows the unavailable elements, not the
        // available ones:
        //if (isElementAvailable(petID)) {
        //logger().log(opencog::Logger::WARN, "Trying to load an OPC that is already available: %s\n", petID.c_str());
        // TODO: send the LOAD SUCCESS message back
        //}
        std::stringstream str;
        //char str[512];
        int opcPort = allocateOpcPort(agentID);
        if (opcPort <= 0) {
            logger().log(opencog::Logger::ERROR, "There is no available socket port to run opc %s\n", agentID.c_str());
            // TODO: notify PROXY about this problem
            return false;
        }
        std::string cmdPrefix;
        std::string cmdSuffix;
        if (atoi(getParameters().get("RUN_OPC_DEBUGGER").c_str()) != 0) {
            cmdPrefix = getParameters().get("OPC_DEBUGGER_PATH");
            cmdPrefix += " ";
            std::cout << "======= YOU MUST ENTER DEBUGGER PARAMETERS ======="
                      << std::endl;
            std::cout << "Please enter the following 2 parameters in "
                      << cmdPrefix << " :" << std::endl;
            std::cout << agentID << " " << ownerID << " " << agentType << " " << agentTraits << " " << opcPort << std::endl;
            str << cmdPrefix << "./opc &";
        } else {
            if (atoi(getParameters().get("CHECK_OPC_MEMORY_LEAKS").c_str()) != 0) {
                cmdPrefix = getParameters().get("VALGRIND_PATH");
                cmdPrefix += " --leak-check=full ";
                cmdSuffix += " > opc.valgrind.memcheck 2>&1";
            } else if (atoi(getParameters().get("CHECK_OPC_MEMORY_USAGE").c_str()) != 0) {
                cmdPrefix = getParameters().get("VALGRIND_PATH");
                cmdPrefix += " --tool=massif --detailed-freq=";
                cmdPrefix += getParameters().get("MASSIF_DETAILED_FREQ");
                cmdPrefix += " --depth=";
                cmdPrefix += getParameters().get("MASSIF_DEPTH");
                cmdPrefix += " ";
                cmdSuffix += " > opc.valgrind.massif 2>&1";
            }
            str << cmdPrefix << "./opc "
            << agentID << " "
            << ownerID << " "
            << agentType << " "
            << agentTraits << " "
            << opcPort << cmdSuffix << " &";
        }
        logger().log(opencog::Logger::INFO, "Starting OPC for %s %s %s (%s) at port %d (command: %s)", agentType.c_str( ), agentID.c_str(), ownerID.c_str(), agentTraits.c_str( ), opcPort, str.str().c_str( ) );

#define USE_SYSTEM_FUNCTION
#ifdef USE_SYSTEM_FUNCTION
        if (atoi(getParameters().get("MANUAL_OPC_LAUNCH").c_str()) == 0) {
            system(str.str().c_str( ) );
        } else {
            printf("\nSpawner Command: %s\n", str.str().c_str());
        }
#else
        int pid = fork();
        if (pid < 0) {
            logger().log(opencog::Logger::ERROR, "Could not fork the spawner process to run opc %s\n", agentID.c_str());
            // TODO: What to do now? Send a NACK back to proxy ???
        } else {
            if (pid == 0) {
                // child proccess
                int execResult = execl("./opc", "opc", agentID.c_str(), ownerID.c_str(), agentType.c_str( ), agentTraits.c_str( ), opcPortStr.c_str(), NULL);
                logger().log(opencog::Logger::DEBUG, "exec method returned %d\n", execResult);
                exit(0); // cannot continue spawner execution...
            }
        }
#endif
    } else if (command == "UNLOAD_AGENT") {
        const std::string& agentID = args.front();

        // request router to erase all messages for the given pet
        std::string cmd("CLEAR_MESSAGE_QUEUE ");
        cmd.append(getID() + " ");
        cmd.append(agentID);
        sendCommandToRouter(cmd);

        // then, unload it
        StringMessage saveExit(getID(), agentID, "SAVE_AND_EXIT");
        logger().log(opencog::Logger::INFO, "Sending SAVE_AND_EXIT message to %s", agentID.c_str());
        sendMessage(saveExit);

        releaseOpcPort(agentID);
    } else {
        logger().log(opencog::Logger::WARN, "Unknown command <%s>. Discarding it", command.c_str());
    }
    return false;
}
