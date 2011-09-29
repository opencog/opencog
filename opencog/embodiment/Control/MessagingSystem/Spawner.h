/*
 * opencog/embodiment/Control/MessagingSystem/Spawner.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef SPAWNER_H
#define SPAWNER_H

#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <exception>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include "EmbodimentCogServer.h"

namespace opencog { namespace messaging {

class Spawner : public EmbodimentCogServer
{

private:
    int minOacPort, maxOacPort;
    std::map<int, std::string> port2PetIdMap;
    std::map<std::string, int> petId2PortMap;

public:

    // ***********************************************/
    // Constructors/destructors

    static BaseServer* createInstance();
    ~Spawner();
    Spawner();
    void init(const std::string &id,
              const std::string &ip, int port) throw (opencog::InvalidParamException, std::bad_exception);

    // ***********************************************/
    // Overrides EmbodimentCogServer interface

    bool processNextMessage(Message *message);

    /**
     * Allocates an available port number, from the range of ports reserved for OACs, to a given pet id.
     * If there is no available port, returns -1.
     */
    int allocateOacPort(const std::string& petId);

    /**
     * Releases the port number allocated to the given pet id, if any
     */
    void releaseOacPort(const std::string& petId);

}; // class

} } // namespace opencog::messaging

#endif
