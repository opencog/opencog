/*
 * opencog/embodiment/Control/MessagingSystem/ServerSocket.h
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


#ifndef SERVERSOCKET_H
#define SERVERSOCKET_H

#include "NetworkElement.h"

#include <boost/thread/thread.hpp>

namespace opencog { namespace messaging {

class ServerSocket
{

private:

    // Used to call-back when Message arrives
    static NetworkElement *master;

    boost::asio::io_service io_service;
    tcp::socket socket;
    boost::thread connectionThread;

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
    ServerSocket();
    void Send(const std::string& cmd);
    static void handle_connection(ServerSocket*);
    void start();
    tcp::socket& getSocket();

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
} } // namespace opencog::messaging

#endif
