/*
 * opencog/embodiment/Control/MessagingSystem/Spawner.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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

#include <opencog/embodiment/Control/LoggerFactory.h>
#include "Spawner.h"
#include "StringMessage.h"
#include <iostream>
#include <sstream>
#include "NetworkElementCommon.h"

using namespace opencog::messaging;
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

void Spawner::init(const std::string &id,
                   const std::string &ip,
                   int port) throw (opencog::InvalidParamException, std::bad_exception)
{

    setNetworkElement(new NetworkElement(id, ip, port));

    minOpcPort = opencog::config().get_int("MIN_OAC_PORT");
    maxOpcPort = opencog::config().get_int("MAX_OAC_PORT");
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

    NetworkElementCommon::parseCommandLine(cmdLine, command, args);
    logger().info("Spawner - line(%s) command(%s) # of parsed arguments(%d).",
                  cmdLine.c_str( ), command.c_str( ), args.size( ) );

    // Order of arguments is:
    // AGENT_ID, OWNER_ID, AGENT_TYPE, AGENT_TRAITS
    if (command == "LOAD_AGENT") {
        std::string agentID = args.front();
        args.pop( );
        std::string ownerID = args.front();
        args.pop( );
        std::string agentType = args.front();
        args.pop( );
        std::string agentTraits = args.front();

        logger().info("Spawner - agentId(%s) ownerId(%s) agentType(%s) agentTraits(%s).",
                      agentID.c_str( ), ownerID.c_str( ), agentType.c_str( ),
                      agentTraits.c_str( ) );

        // TODO: Find a way to figure out that OAC is already running.
        // The call to isElementAvailable method does not work because it only
        // knows the unavailable elements, not the available ones:
        //if (isElementAvailable(petID)) {
        //logger().warn("Trying to load an OAC that is already available: %s\n", petID.c_str());
        // TODO: send the LOAD SUCCESS message back
        //}
        std::stringstream command_ss;
        //char str[512];
        int opcPort = allocateOpcPort(agentID);
        if (opcPort <= 0) {
            logger().error("There is no available socket port to run opc %s\n", agentID.c_str());
            // TODO: notify PROXY about this problem
            return false;
        }
        // Calculates the CogServer shell  and ZeroMQ publish port to be used 
        // (based on the opcPort offset)
        int portOffset = opcPort - minOpcPort + 1;
        int cogServerShellPort = opencog::config().get_int("SERVER_PORT") + portOffset;
        int zmqPublishPort = opencog::config().get_int("ZMQ_PUBLISH_PORT") + portOffset;

        std::string cmdPrefix;
        std::string cmdSuffix;

        std::string agentArgs = 
            agentID + " " +
            ownerID + " " + 
            agentType + " " +
            agentTraits + " " + 
            boost::lexical_cast<std::string>(opcPort) + " " +
            boost::lexical_cast<std::string>(cogServerShellPort) + " " +
            boost::lexical_cast<std::string>(zmqPublishPort); 

        if (opencog::config().get_bool("RUN_OAC_DEBUGGER")) {
            std::string debuggerPath = opencog::config().get("OAC_DEBUGGER_PATH");
            cmdPrefix = debuggerPath + " ";
            if(opencog::config().get_bool("PASS_OAC_ARG_DEBUGGER_COMMAND")) {
                command_ss << cmdPrefix << " ./opc " << agentArgs << " ";

                if(opencog::config().get_bool("ENABLE_UNITY_CONNECTOR")) {
                    // This is a hack. To redirect "PROXY_ID" setting to agent id.
                    // There are also some tweaks in OAC code.
                    command_ss << message->getFrom() << " " << cmdSuffix << " &";
                } else {
                    command_ss << cmdSuffix << " &";
                }

            }
            else {
                command_ss << cmdPrefix << " ./opc ";
                std::cout << "====== YOU MUST ENTER DEBUGGER PARAMETERS ======"
                          << std::endl;
                std::cout << "Please enter the following 2 parameters in "
                          << debuggerPath << " :" << std::endl;
                std::cout << agentArgs << std::endl;
            }
        } else {
            if (opencog::config().get_bool("CHECK_OAC_MEMORY_LEAKS")) {
                cmdPrefix = opencog::config().get("VALGRIND_PATH");
                cmdPrefix += " --leak-check=full ";
                cmdSuffix += " > opc.valgrind.memcheck 2>&1";
            } else if (opencog::config().get_bool("CHECK_OAC_MEMORY_USAGE")) {
                cmdPrefix = opencog::config().get("VALGRIND_PATH");
                cmdPrefix += " --tool=massif --detailed-freq=";
                cmdPrefix += opencog::config().get("MASSIF_DETAILED_FREQ");
                cmdPrefix += " --depth=";
                cmdPrefix += opencog::config().get("MASSIF_DEPTH");
                cmdPrefix += " ";
                cmdSuffix += " > opc.valgrind.massif 2>&1 &";
            }
            command_ss << cmdPrefix << "./opc " << agentArgs << " ";

			if(opencog::config().get_bool("ENABLE_UNITY_CONNECTOR")) {
				// This is a hack. To redirect "PROXY_ID" setting to agent id.
				// There are also some tweaks in OAC code.
				command_ss << message->getFrom() << " " << cmdSuffix << " &";
			} else {
				command_ss << cmdSuffix << " &";
			}
        }
        logger().info("Starting OAC for %s %s %s (%s) at NE port %d; Shell port %d; ZeroMQ publish port %d; (command: %s)", 
                      agentType.c_str( ), 
                      agentID.c_str(),
                      ownerID.c_str(),
                      agentTraits.c_str( ), 
                      opcPort, 
                      cogServerShellPort, 
                      zmqPublishPort, 
                      command_ss.str().c_str( ) 
                     );

        if (!opencog::config().get_bool("MANUAL_OAC_LAUNCH")) {
            system(command_ss.str().c_str( ) );
        } else {
            printf("\nSpawner Command: %s\n", command_ss.str().c_str());
        }
    } else if (command == "UNLOAD_AGENT") {
        const std::string& agentID = args.front();

        // request router to erase all messages for the given pet
        std::string cmd("CLEAR_MESSAGE_QUEUE ");
        cmd.append(getID() + " ");
        cmd.append(agentID);
        sendCommandToRouter(cmd);

        // then, unload it
        StringMessage saveExit(getID(), agentID, "SAVE_AND_EXIT");
        logger().info("Sending SAVE_AND_EXIT message to %s", agentID.c_str());
        sendMessage(saveExit);

        releaseOpcPort(agentID);
    } else {
        logger().warn("Unknown command <%s>. Discarding it", command.c_str());
    }
    return false;
}
