/*
 * opencog/embodiment/PetaverseProxySimulator/WorldSimulator.h
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

#ifndef WORLDSIMULATOR_H
#define WORLDSIMULATOR_H

#include <vector>
#include <PetAction.h>

namespace PetaverseProxySimulator {

class WorldSimulator {

    private:

    public:

        // ***********************************************/
        // Constructors/destructors

        virtual ~WorldSimulator();
        WorldSimulator();

        /**
         * Creates a new agent into the simulated world
	 * @param name of the agent to be created
	 * @param type of the agent to be created ("pet" or "avatar")
	 * @param echoing if all messages related to this agents must be logged. 
         * Default implementation: Just returns the passed name. 
         */
        virtual std::string createAgent(const std::string& name, const std::string& type, float x, float y,  bool echoing);

        /**
         * Issue a time tick to the simulated world to get new perceptions
         */
        virtual void timeTick();

        /**
         * Execute the given pet action in the simulated world.
         * @return 0 if the command was already executed with success. 
	 *         ULONG_MAX if the command already failed. 
	 *         Otherwise returns any other positive number that will identify the action 
	 *         for further notification of its status via callback methods.
         *
         * Default implementation: Just printf commands and args and returns 0.
         */
        virtual unsigned long executeAction(const std::string& agentName, const PerceptionActionInterface::PetAction &petAction);

}; // class
}  // namespace

#endif
