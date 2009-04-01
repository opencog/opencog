/*
 * opencog/embodiment/Control/MessagingSystem/PetInterfaceUpdaterAgent.cc
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

#include <vector>

#include <opencog/atomspace/AtomSpace.h>

#include <Sockets/SocketHandler.h>
#include <Sockets/ISocketHandler.h>
#include <Sockets/TcpSocket.h>

#include "AtomSpaceUtil.h"
#include "OPC.h"
#include "PetInterfaceUpdaterAgent.h"
#include <opencog/spatial/LocalSpaceMap2D.h>

#include "util/StringManipulator.h"

using namespace OperationalPetController;
using namespace Spatial;

class InterfaceUpdaterClientSocket : public TcpSocket {

public:

    std::string text;

    InterfaceUpdaterClientSocket(ISocketHandler& h,const std::string& cmd) : 
                                 TcpSocket(h) ,text(cmd) {
        SetLineProtocol();
    }

    ~InterfaceUpdaterClientSocket() {
    }

    void OnConnect() {
        Send(text + "\n");
    }

    void OnLine(const std::string& line) {
        logger().log(opencog::Logger::DEBUG, 
                        "PetInterfaceUpdaterAgent - onLine(%s)", line.c_str());
        if (! line.size()) {
            SetCloseAndDelete();
        }
    }

};

PetInterfaceUpdaterAgent::~PetInterfaceUpdaterAgent() {
}

PetInterfaceUpdaterAgent::PetInterfaceUpdaterAgent() {
    interfacePortNumber = 11354; // Remember to change PetInterface.tcl if you 
                                 // change this port number
}

void PetInterfaceUpdaterAgent::startGUI() {
    system("wish -f ./PetInterface.tcl &");
}

void PetInterfaceUpdaterAgent::run(opencog::CogServer *server) {
    
    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterAgent - run()");

    OPC *opc = (OPC *) server;
    updateLocalMap(opc);
    updateFeelings(opc);
    updateLastSelectedAction(opc);
}

void PetInterfaceUpdaterAgent::sendLine(const std::string &line) {
    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterAgent - sendLine()");
    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterAgent - line = %s", line.c_str());
    SocketHandler h;
    InterfaceUpdaterClientSocket cc(h, line);
    cc.Open("127.0.0.1", interfacePortNumber);
    h.Add(&cc);
    h.Select(1,0);
    while (h.GetCount()) {
        h.Select(1,0);
    }
    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterAgent - sendLine() finished");
}

void PetInterfaceUpdaterAgent::updateLastSelectedAction(OPC *opc) {

    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterAgent - updateLastSelectedAction()");

    std::string line = "LASTSELECTEDGOAL ";
    line.append("No goals");
    sendLine(line);

    line = "LASTSELECTEDSCHEMA ";
    std::string lastSelectedSchema = opc->getRuleEngine().getCurrentAction(); 
    if (lastSelectedSchema.empty()) {
        line.append("none");
    } else {
        line.append(lastSelectedSchema);
    }

    sendLine(line);
}



void PetInterfaceUpdaterAgent::updateFeelings(OPC *opc) {

    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterAgent - updateFeelings()");

    std::vector<std::string> allFeelingNames;
    Handle petHandle = opc->getAtomSpace().getHandle(SL_PET_NODE, opc->getPet().getName());
    if ( petHandle == Handle::UNDEFINED ) {
      petHandle = opc->getAtomSpace().getHandle(SL_HUMANOID_NODE, opc->getPet().getName());
    } // if

    std::string hunger = opencog::toString(AtomSpaceUtil::getPredicateValue(opc->getAtomSpace(), "hunger", petHandle));
    std::string thirst = opencog::toString(AtomSpaceUtil::getPredicateValue(opc->getAtomSpace(), "thirst", petHandle));

    allFeelingNames.push_back(hunger);
    allFeelingNames.push_back(thirst);
    //allFeelingNames.push_back(GoalsActionsImportance::OWNER_SATISFACTION);

    for (unsigned int i = 0; i < allFeelingNames.size(); i++) {
        Handle goalHandle = opc->getAtomSpace().getHandle(FEELING_NODE, allFeelingNames[i]);
        float strength = opc->getAtomSpace().getTV(goalHandle).getMean();
        std::string line = "FEELING ";
        line.append(allFeelingNames[i]);
        line.append(" ");
        char s[128];
        sprintf(s, "%f", strength);
        line.append(s);
        sendLine(line);
    }
}

void PetInterfaceUpdaterAgent::updateLocalMap(OPC *opc) {

    if (! opc->getSpaceServer().isLatestMapValid()) {
        return;
    }

    const SpaceServer::SpaceMap& latestMap = opc->getSpaceServer().getLatestMap();
    //HandleSeq objectHandles;
    std::vector<std::string> objectIds;
    latestMap.getAllObjects(back_inserter(objectIds));
    std::string line = "LOCALMAP ";
    logger().log(opencog::Logger::DEBUG, "num objects = %d", objectIds.size());
    for (unsigned int i = 0; i < objectIds.size(); i++) {
        logger().log(opencog::Logger::FINE, "%s:", objectIds[i].c_str());
        line.append(objectIds[i]);
        line.append(" ");

        try {
	  const std::vector<Spatial::GridPoint>& perimeter = 
	    latestMap.getObjectPoints(objectIds[i]);

            for (unsigned int j = 0; j < perimeter.size(); j++) {
	      Spatial::Point realPoint = latestMap.unsnap( perimeter[j] );

	      logger().log(opencog::Logger::FINE, "(%f,%f)", 
			      realPoint.first, realPoint.second);
                char s[128];
                sprintf(s, "%f", realPoint.first);
                line.append(s);
                line.append(" ");
                sprintf(s, "%f", realPoint.second);
                line.append(s);
                if (j != (perimeter.size() - 1)) {
                    line.append(" ");
                }
            }
        } catch (opencog::AssertionException& e){
            logger().log(opencog::Logger::ERROR, "Object '%s' has no points.", 
                            objectIds[i].c_str());
        }

        if (i != (objectIds.size() - 1)) {
            line.append("\t");
        }
    }

    sendLine(line);
}

