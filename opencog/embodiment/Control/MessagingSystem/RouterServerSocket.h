/*
 * opencog/embodiment/Control/MessagingSystem/RouterServerSocket.h
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


#ifndef ROUTERSERVERSOCKET_H
#define ROUTERSERVERSOCKET_H

#include "Router.h"

#include <boost/thread/thread.hpp>
#include <boost/regex.hpp>

namespace opencog { namespace messaging {

class RouterServerSocket
{

private:

    int currentState;
    static const int WAITING_COMMAND = 1;
    static const int READING_MESSAGE = 2;

    // Used for NEW_MESSAGE command, to keep track of the src/dst,
    // type and size of the message transmitted
    std::string currentMessageText;
    std::string currentMessageFrom;
    std::string currentMessageTo;
    int currentMessageType;
    int currentMessageSize;
    int messageLinesCountdown;
    bool firstLineOfMessageFlag;

    // Used to call-back when messages arrive
    static Router *master;

    boost::asio::io_service io_service;
    tcp::socket socket;
    boost::thread connectionThread;

    void sendAnswer(const std::string &msg);
    void addNetworkElement(const std::string &id, const std::string &ip, int port);

    // Send messages in queue to network element id. If limit is
    // negative then all messages are sent, otherwise only 'limit'
    // messages are sent.
    void sendRequestedMessages(const std::string &id, int limit);

    void storeNewMessage();

public:

    // ***********************************************/
    // Constructors/destructors

    ~RouterServerSocket();
    RouterServerSocket();
    void Send(const std::string& cmd);
    static void handle_connection(RouterServerSocket*);
    void start();
    tcp::socket& getSocket();

    // ***********************************************/
    // General

    /**
     * Sets the class that will be notified by call-back regarding communication events.
     */
    static void setMaster(Router *router);

    // ***********************************************/
    // Overriden from TcpSocket

    void OnLine(const std::string &line);

}; // class
} } // namespace opencog::messaging

#endif
