/*
 * opencog/embodiment/Control/MessagingSystem/DummyServer.h
 *
 * Copyright (C) 2007-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna
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

#ifndef DUMMYSERVER_H
#define DUMMYSERVER_H

#include <string>
#include <SystemParameters.h>
#include "Message.h"
#include "NetworkElement.h"

namespace MessagingSystem {

class DummyServer : public NetworkElement {

    private:

        int cycleCount;
        int petCount;
        Control::SystemParameters parameters;

    public:

        // ***********************************************/
        // Constructors/destructors

        ~DummyServer();
        DummyServer(const Control::SystemParameters &params, const std::string &id, const std::string &ip, int port);

        // ***********************************************/
        // Overwritten from NetworkElement

        /**
         * This method implements the API of this server (in terms of the Messages it
         * expects from its clients. It is recommended that this API is documented in
         * the header of each server.
         *
         * DummyServer manages only two kinds of messages:
         *
         * (1) Shutdown message
         *     Syntax: SHUTDOWN
         *     Args: (no args)
         *
         * (2) Sleep message
         *     Syntax: SLEEP <N>
         *     Args: N - integer, the number of seconds the server will sleep
         */
        bool processNextMessage(Message *message) ;

        /**
         * This method may be overwritten to make the server behaves in a cycle-based manner.
         * (supposing that serverLoop() was called to control this element, which is
         * stringly recommended)
         *
         * It is VERY IMPORTANT to note that Messages retrieval (from router) is requested
         * in the default (NetworkElement::idleTime()). It means that if you overwrites this
         * method you need to provide a policy to decide when messages will be retrieved from
         * router.
         *
         * Once messages retrieval is requested, the main thread will continue and messages will
         * be delivered assynchronously. serverLoop() will known when those messages actually arrive
         * so it call processNextMessage() accordingly.
         *
         * Try to keep idleTime() processing time as low as possible. Allowing this element to
         * manage its message queue reducing answering latency time as perceived by clients.
         */
        void idleTime();

}; // class
}  // namespace

#endif
