/**
 * SleepAgent.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 12:36:43 BRT 2007
 */

#include "SleepAgent.h"

using namespace OperationalPetController;

SleepAgent::~SleepAgent() {
}

SleepAgent::SleepAgent() {
}

void SleepAgent::run(opencog::CogServer *server) {
    
	logger().log(opencog::Logger::FINE, "SleepAgent - Sleeping for one second.");
    sleep(1);
}
