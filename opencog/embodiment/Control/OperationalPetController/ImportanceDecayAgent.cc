/**
 * ImportanceDecayAgent.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 11:30:53 BRT 2007
 */

#include "OPC.h"
#include "ImportanceDecayAgent.h"

using namespace OperationalPetController;

ImportanceDecayAgent::~ImportanceDecayAgent() {
}

ImportanceDecayAgent::ImportanceDecayAgent() {
    lastTickTime = 0;
}

void ImportanceDecayAgent::run(opencog::CogServer *server) {

    logger().log(opencog::Logger::FINE, 
                    "ImportanceDecayTask - Executing decayShortTermImportance().");
    ((OPC *) server)->decayShortTermImportance();

}
