/**
 * PetInterfaceUpdaterTask.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Thu Oct 18 12:46:07 BRDT 2007
 */

#include <vector>

#include <opencog/atomspace/AtomSpace.h>

#include <Sockets/SocketHandler.h>
#include <Sockets/ISocketHandler.h>
#include <Sockets/TcpSocket.h>

#include "AtomSpaceUtil.h"
#include "OPC.h"
#include "PetInterfaceUpdaterTask.h"
#include "LocalSpaceMap2D.h"

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
                        "PetInterfaceUpdaterTask - onLine(%s)", line.c_str());
        if (! line.size()) {
            SetCloseAndDelete();
        }
    }

};

PetInterfaceUpdaterTask::~PetInterfaceUpdaterTask() {
}

PetInterfaceUpdaterTask::PetInterfaceUpdaterTask() {
    interfacePortNumber = 11354; // Remember to change PetInterface.tcl if you 
                                 // change this port number
}

void PetInterfaceUpdaterTask::startGUI() {
    system("wish -f ./PetInterface.tcl &");
}

void PetInterfaceUpdaterTask::run(MessagingSystem::NetworkElement *ne) {
    
    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterTask - run()");

    OPC *opc = (OPC *) ne;
    updateLocalMap(opc);
    updateFeelings(opc);
    updateLastSelectedAction(opc);
}

void PetInterfaceUpdaterTask::sendLine(const std::string &line) {
    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterTask - sendLine()");
    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterTask - line = %s", line.c_str());
    SocketHandler h;
    InterfaceUpdaterClientSocket cc(h, line);
    cc.Open("127.0.0.1", interfacePortNumber);
    h.Add(&cc);
    h.Select(1,0);
    while (h.GetCount()) {
        h.Select(1,0);
    }
    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterTask - sendLine() finished");
}

void PetInterfaceUpdaterTask::updateLastSelectedAction(OPC *opc) {

    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterTask - updateLastSelectedAction()");

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



void PetInterfaceUpdaterTask::updateFeelings(OPC *opc) {

    logger().log(opencog::Logger::DEBUG, "PetInterfaceUpdaterTask - updateFeelings()");

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

void PetInterfaceUpdaterTask::updateLocalMap(OPC *opc) {

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

