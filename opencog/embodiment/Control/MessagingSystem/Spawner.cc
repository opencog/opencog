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

using std::string;
using boost::lexical_cast;
using namespace opencog;
using namespace messaging;

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

void Spawner::init(const string &id,
                   const string &ip,
                   int port) throw (InvalidParamException, std::bad_exception)
{

    setNetworkElement(new NetworkElement(id, ip, port));

    minOpcPort = config().get_int("MIN_OAC_PORT");
    maxOpcPort = config().get_int("MAX_OAC_PORT");
    int maxNumberOfPets = maxOpcPort - minOpcPort;

    if (minOpcPort <= 0 || maxOpcPort <= 0 || maxNumberOfPets < 0) {
        throw InvalidParamException(TRACE_INFO,
                                             "Spawner - Invalid port range defined by (Min '%d', Max '%d').",
                                             minOpcPort, maxOpcPort);
    }
}

extern unsigned sleep(unsigned useconds);

int Spawner::allocateOpcPort(const string& petId)
{
    std::map<string, int>::const_iterator itr = petId2PortMap.find(petId);
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

void Spawner::releaseOpcPort(const string& petId)
{
    std::map<string, int>::const_iterator itr = petId2PortMap.find(petId);
    if (itr != petId2PortMap.end()) {
        port2PetIdMap.erase(itr->second);
        petId2PortMap.erase(petId);
    }
}

bool Spawner::processNextMessage(Message *message)
{

    string cmdLine = message->getPlainTextRepresentation();
    string command;
    std::queue<string> args;

    NetworkElementCommon::parseCommandLine(cmdLine, command, args);
    logger().info("Spawner - line(%s) command(%s) # of parsed arguments(%d).",
                  cmdLine.c_str( ), command.c_str( ), args.size( ) );

    // Order of arguments is:
    // AGENT_ID, OWNER_ID, AGENT_TYPE, AGENT_TRAITS
    if (command == "LOAD_AGENT") {
        string agentID = args.front();
        args.pop( );
        string ownerID = args.front();
        args.pop( );
        string agentType = args.front();
        args.pop( );
        string agentTraits = args.front();

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
        int cogServerShellPort = config().get_int("SERVER_PORT") + portOffset;
        int zmqPublishPort = config().get_int("ZMQ_PUBLISH_PORT") + portOffset;

        string cmdPrefix;
        string cmdSuffix;

        string agentArgs = 
            agentID + " " +
            ownerID + " " + 
            agentType + " " +
            agentTraits + " " + 
            lexical_cast<string>(opcPort) + " " +
            lexical_cast<string>(cogServerShellPort) + " " +
            lexical_cast<string>(zmqPublishPort); 

        if (config().get_bool("RUN_OAC_DEBUGGER")) {
            string debuggerPath = config().get("OAC_DEBUGGER_PATH");
            cmdPrefix = debuggerPath + " ";
            if(config().get_bool("PASS_OAC_ARG_DEBUGGER_COMMAND")) {
                command_ss << cmdPrefix << " ./opc " << agentArgs << " ";

                if(config().get_bool("ENABLE_UNITY_CONNECTOR")) {
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
            if (config().get_bool("CHECK_OAC_MEMORY_LEAKS")) {
                cmdPrefix = config().get("VALGRIND_PATH");
                cmdPrefix += " --leak-check=full ";
                cmdSuffix += " > opc.valgrind.memcheck 2>&1";
            } else if (config().get_bool("CHECK_OAC_MEMORY_USAGE")) {
                cmdPrefix = config().get("VALGRIND_PATH");
                cmdPrefix += " --tool=massif --detailed-freq=";
                cmdPrefix += config().get("MASSIF_DETAILED_FREQ");
                cmdPrefix += " --depth=";
                cmdPrefix += config().get("MASSIF_DEPTH");
                cmdPrefix += " ";
                cmdSuffix += " > opc.valgrind.massif 2>&1 &";
            }
            command_ss << cmdPrefix << "./opc " << agentArgs << " ";

			if(config().get_bool("ENABLE_UNITY_CONNECTOR")) {
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

        if (!config().get_bool("MANUAL_OAC_LAUNCH")) {
            system(command_ss.str().c_str( ) );
        } else {
            printf("\nSpawner Command: %s\n", command_ss.str().c_str());
        }
    } else if (command == "UNLOAD_AGENT") {
        const string& agentID = args.front();

        // request router to erase all messages for the given pet
        string cmd("CLEAR_MESSAGE_QUEUE ");
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
