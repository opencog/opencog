/**
 * WorldSimulator.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Oct  3 21:36:22 BRT 2007
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
