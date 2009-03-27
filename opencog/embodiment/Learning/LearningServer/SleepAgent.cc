/**
 * SleepAgent.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 12:36:43 BRT 2007
 */
#include "SleepAgent.h"

using namespace LearningServer;
using namespace opencog;

SleepAgent::~SleepAgent() {
}

SleepAgent::SleepAgent() {
}

void SleepAgent::run(CogServer *ls) {
    logger().log(opencog::Logger::FINE,
                 "SleepAgent - Executing SleepAgent.");
  sleep(2);
}
