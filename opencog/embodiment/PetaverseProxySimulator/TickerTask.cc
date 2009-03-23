/**
 * TickerTask.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Fri Oct  5 23:22:43 BRT 2007
 */

#include "TickerTask.h"
#include "PVPSimulator.h"
#include "SimulationParameters.h"

#include <time.h>

using namespace PetaverseProxySimulator;

TickerTask::~TickerTask() {
}

TickerTask::TickerTask(SimulationParameters& _simParameters) : simParameters(_simParameters) {
    lastTickTime = 0;
    realTimeSecondsInOneTick = atoi(simParameters.get("REAL_TIME_SECONDS_IN_ONE_TICK").c_str());
}

void TickerTask::run(MessagingSystem::NetworkElement *ne) {
    time_t currentTime = time(NULL);
    if ((currentTime- lastTickTime) >= realTimeSecondsInOneTick) {
        ((PVPSimulator *) ne)->timeTick();
        lastTickTime = currentTime;
    }
}

