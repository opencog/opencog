/*
 * opencog/embodiment/PetaverseProxySimulator/PVPSimulatorExecutable.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#include <SystemParameters.h>
#include <exception>
#include "util/exceptions.h"
#include "PVPSimulator.h"
#include "SimulationParameters.h"
#include "TickerAgent.h"
#include "InterfaceListenerAgent.h"
#include "MessageSenderAgent.h"
#include "util/files.h"
#include <StringMessage.h>
#include <unistd.h>

using namespace PetaverseProxySimulator;
int main(int argc, char *argv[])
{

    Control::SystemParameters parameters;
    if (fileExists(parameters.get("CONFIG_FILE").c_str())) {
        parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    SimulationParameters simParameters;
    if (fileExists(simParameters.get("CONFIG_FILE").c_str())) {
        simParameters.loadFromFile(simParameters.get("CONFIG_FILE"));
    }
    //PVPSimulator simulator(parameters, simParameters, parameters.get("PROXY_ID"), "127.0.0.1", 8211);

    server(PVPSimulator::createInstance);
    PVPSimulator& simulator = static_cast<PVPSimulator&>(server());
    simulator.init(parameters, simParameters, parameters.get("PROXY_ID"), "127.0.0.1", 16315);

    if (!simulator.connectToSimWorld()) {
        logger().log(opencog::Logger::ERROR, "Could not connect to the simulated World\n");
        exit(0);
    }

    Factory<TickerAgent, Agent> tickerAgentFactory;
    simulator.registerAgent(TickerAgent::info().id, &tickerAgentFactory);
    simulator.createAgent(TickerAgent::info().id, true);

    Factory<InterfaceListenerAgent, Agent> interfaceListenerAgentFactory;
    simulator.registerAgent(InterfaceListenerAgent::info().id, &interfaceListenerAgentFactory);
    simulator.createAgent(InterfaceListenerAgent::info().id, true);

    Factory<MessageSenderAgent, Agent> messageSenderAgentFactory;
    simulator.registerAgent(MessageSenderAgent::info().id, &messageSenderAgentFactory);
    simulator.createAgent(MessageSenderAgent::info().id, true);

    simParameters.startSimulation();

    try {
        simulator.serverLoop();
    } catch (std::bad_alloc) {
        logger().log(opencog::Logger::ERROR, "PVPSimExec - PVPSim raised a bad_alloc exception.");
        simulator.persistState();

    } catch (...) {
        logger().log(opencog::Logger::ERROR,
                     "PVPSimExec - An exceptional situation occured. Check log for information.");
        simulator.persistState();
    }

    return 0;
}

