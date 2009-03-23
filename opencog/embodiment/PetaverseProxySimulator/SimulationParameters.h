/**
 * SimulationParameters.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Sat Oct  6 21:08:02 BRT 2007
 */

#ifndef SIMULATIONPARAMETERS_H
#define SIMULATIONPARAMETERS_H

#include <time.h>
#include "SystemParameters.h"

namespace PetaverseProxySimulator {

class SimulationParameters : public Control::SystemParameters {

    public:

        SimulationParameters();
        ~SimulationParameters();

        time_t simulationStart;
        time_t simulationTicks;

        void startSimulation();
        int getCurrentSimulationSeconds();
        void timeTick();

}; // class
}  // namespace

#endif
