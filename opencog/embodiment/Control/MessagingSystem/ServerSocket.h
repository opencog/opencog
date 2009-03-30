/*
 * opencog/embodiment/Control/MessagingSystem/ServerSocket.h
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

#ifndef SERVERSOCKET_H
#define SERVERSOCKET_H

#include "NetworkElement.h"

#include <Sockets/TcpSocket.h>
#include <Sockets/ISocketHandler.h>

namespace MessagingSystem {

class ServerSocket : public TcpSocket {

    private:

        // Used to call-back when Message arrives
        static NetworkElement *master;

        // States used in in the state machine inside onLine()
        static const int DOING_NOTHING = 0;
        static const int READING_MESSAGES = 1;
        int currentState;

        std::string currentMessage;
        std::string currentMessageFrom;
        std::string currentMessageTo;
        int currentMessageType;
        int lineCount;

    public:

        // ***********************************************/
        // Constructors/destructors

        ~ServerSocket();
        ServerSocket(ISocketHandler &handler);

        // ***********************************************/
        // General

        /**
         * Sets the class that will be notified by call-back regarding communication events.
         */
        static void setMaster(NetworkElement *ne);
          
        // ***********************************************/
        // Overriden from TcpSocket
          
        void OnLine(const std::string&);

}; // class
}  // namespace

#endif
