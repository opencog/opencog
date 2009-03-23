/**
 * WorldSimulator.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Oct  3 21:36:22 BRT 2007
 */

#include "WorldSimulator.h"
#include "util/Logger.h"

using namespace PetaverseProxySimulator;
using namespace opencog;

WorldSimulator::~WorldSimulator() {
}

WorldSimulator::WorldSimulator() {
}

std::string WorldSimulator::createAgent(const std::string& name, const std::string& type, float x, float y, bool echoing) {
    logger().log(opencog::Logger::INFO, "Created agent %s in the sim world", name.c_str());
    return name;
}
    
void WorldSimulator::timeTick() {
    logger().log(opencog::Logger::INFO, "Time tick to sim world");
}
    
unsigned long WorldSimulator::executeAction(const std::string& agentName, const PerceptionActionInterface::PetAction &petAction) {
    logger().log(opencog::Logger::WARN, "_PVPSimulator_UNIT_TEST_TAG_ Executing action for agent '%s': %s", 
                    agentName.c_str(), petAction.stringRepresentation().c_str());
    //return 0; Cannot return 0 because this would require NetworkElement for sending action status for this...
    return ULONG_MAX - 1;
}

