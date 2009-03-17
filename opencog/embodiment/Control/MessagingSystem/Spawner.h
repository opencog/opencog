/**
 * Spawner.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Sat Jun 23 22:26:49 BRT 2007
 */

#ifndef SPAWNER_H
#define SPAWNER_H

#include <SystemParameters.h>
#include <exception>
#include <LADSUtil/Logger.h>

#include <LADSUtil/exceptions.h>
#include "NetworkElement.h"

namespace MessagingSystem {

class Spawner : public NetworkElement {

    private:
        int minOpcPort, maxOpcPort;
	std::map<int, std::string> port2PetIdMap;
	std::map<std::string, int> petId2PortMap;

    public:

        // ***********************************************/
        // Constructors/destructors

        ~Spawner();
        Spawner(const Control::SystemParameters &params, const std::string &id, 
                const std::string &ip, int port) throw (LADSUtil::InvalidParamException, std::bad_exception);

        // ***********************************************/
        // Overrides NetworkElement interface

        bool processNextMessage(Message *message);

	/**
	 * Allocates an available port number, from the range of ports reserved for OPCs, to a given pet id.
	 * If there is no available port, returns -1.
	 */
        int allocateOpcPort(const std::string& petId);

        /**
	 * Releases the port number allocated to the given pet id, if any
	 */
        void releaseOpcPort(const std::string& petId);
        
}; // class
}  // namespace

#endif
