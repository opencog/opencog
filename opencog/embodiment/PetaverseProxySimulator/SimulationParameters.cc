/**
 * SimulationParameters.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Sat Oct  6 21:08:02 BRT 2007
 */

#include "SimulationParameters.h"
#include "stdlib.h"

using namespace PetaverseProxySimulator;

SimulationParameters::SimulationParameters() {
    simulationStart = 0;
    simulationTicks = 0;

    table["CONFIG_FILE"] = "pvpsim.cfg";

    // Number of simulated minutes each tick represents
    table["DEFAULT_SIMULATION_MINUTES_PER_TICK"] = "1";
    // How many elapsed real seconds would be needed for a new simulation tick be generated.
    table["REAL_TIME_SECONDS_IN_ONE_TICK"] = "2";

    // Flag to enable/disable the generation of gold standards (for using in automated tests).
    table["GENERATE_GOLD_STANDARD"] = "0";
    // Name of the file where the gold standards will be saved, if the previous parameter is enabled.
    table["GOLD_STANDARD_FILENAME"] = "PVPSimGoldStandard.txt";
}

SimulationParameters::~SimulationParameters() {
}

int SimulationParameters::getCurrentSimulationSeconds() {
    //return simulationStart + (int) (simulationTicks * atof(get("DEFAULT_SIMULATION_MINUTES_PER_TICK").c_str()) * 60);
    return (int) (simulationTicks * atof(get("DEFAULT_SIMULATION_MINUTES_PER_TICK").c_str()) * 60);
}

void SimulationParameters::timeTick() {
    simulationTicks++;
}

void SimulationParameters::startSimulation() {
    simulationStart = time(NULL);
}
