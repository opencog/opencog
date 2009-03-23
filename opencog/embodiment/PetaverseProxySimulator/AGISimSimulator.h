#ifndef AGISIM_SIMULATOR_H
#define AGISIM_SIMULATOR_H
/**
 * AGISimSimulator.h
 *
 * Author: Welter Luigi
 */

#include "WorldSimulator.h"
#include "SimProxy.h"

namespace PetaverseProxySimulator {

class AGISimSimulator : public WorldSimulator {

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
