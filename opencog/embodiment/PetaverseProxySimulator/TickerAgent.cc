/**
 * TickerAgent.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Fri Oct  5 23:22:43 BRT 2007
 */

#include "TickerAgent.h"
#include "PVPSimulator.h"
#include "SimulationParameters.h"

#include <time.h>

using namespace PetaverseProxySimulator;

TickerAgent::~TickerAgent() {
}

TickerAgent::TickerAgent() {
}

void TickerAgent::init(SimulationParameters& _simParameters) {
    simParameters = &_simParameters;
    lastTickTime = 0;
    realTimeSecondsInOneTick = atoi(simParameters->get("REAL_TIME_SECONDS_IN_ONE_TICK").c_str());
}

void TickerAgent::run(opencog::CogServer *server) {
    time_t currentTime = time(NULL);
    if ((currentTime- lastTickTime) >= realTimeSecondsInOneTick) {
        ((PVPSimulator *) server)->timeTick();
        lastTickTime = currentTime;
    }
}

