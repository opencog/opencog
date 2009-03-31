/*
 * opencog/embodiment/PetaverseProxySimulator/SimulationParameters.cc
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
