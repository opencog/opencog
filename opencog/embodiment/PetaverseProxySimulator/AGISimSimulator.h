/*
 * opencog/embodiment/PetaverseProxySimulator/AGISimSimulator.h
 *
 * Copyright (C) 2007-2008 Welter Luigi
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
#ifndef AGISIM_SIMULATOR_H
#define AGISIM_SIMULATOR_H

#include "WorldSimulator.h"
#include "SimProxy.h"

namespace PetaverseProxySimulator
{

class AGISimSimulator : public WorldSimulator
{

private:
    typedef std::map<std::string, SimProxy*> AgentToSimProxyMap;
    AgentToSimProxyMap simProxyMap;

    std::string host;
    int nextPort;
    AsynchronousPerceptionAndStatusHandler* petPerceptionHandler;
    SimProxy* petSimProxy;

    SimProxy* getConnectedSimProxy(const std::string& agentName);

public:

    /**
     * Constructor
     * @param host The name of the host where AGISimSim server is running
     * @param port The socket port where AGISimSim server listen for connections
     * @param handler The object that implements the AsynchronousPerceptionAndStatusHandler interface for the pet
     */
    AGISimSimulator(const char* host, AsynchronousPerceptionAndStatusHandler* petPerceptionHandler);

    /**
     * Destructor
     */
    ~AGISimSimulator();

    /**
     * Overrides the default implementation
     */
    std::string createAgent(const std::string& name, const std::string& type, float x, float y, bool echoing);

    /**
     * Overrides the default implementation
     */
    void timeTick();

    /**
     * Overrides the default implementation
     */
    unsigned long executeAction(const std::string& agentName, const PerceptionActionInterface::PetAction &petAction);

}; // class
}  // namespace

#endif
