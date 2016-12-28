/*
 * opencog/cogserver/server/NetworkServer.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_SIMPLE_NETWORK_SERVER_H
#define _OPENCOG_SIMPLE_NETWORK_SERVER_H

#include <string>
#include <queue>
#include <thread>

#include <boost/asio.hpp>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

/**
 * This class implements the entity responsible for managing the
 * cogserver's network server.
 *
 * The network server runs on its own thread (thus freeing the cogserver's main
 * loop to deal with requests and agents only). It may be enabled/disabled
 * at will so that a cogserver may run in networkless mode if desired.
 *
 * The network server supports only one server socket. Client applications
 * should use the 'start' methodr to start listening to a port.
 * Currently, the network server doesn't
 * support selecting the network interface that the server socket will bind to
 * (every server sockets binds to 0.0.0.0, i.e., all interfaces). Thus,
 * server sockets are identified/selected by the port they bind to.
 */
class NetworkServer
{
protected:
    bool _running;
    short _port;
    boost::asio::io_service _io_service;
    boost::asio::ip::tcp::acceptor _acceptor;
    std::thread* _listener_thread;

    /** The network server's main listener thread.  */
    void listen();

public:

    /**
     * Starts the NetworkServer in a new thread.
     * The socket listen happens in the new thread.
     */
    NetworkServer(unsigned short port);
    ~NetworkServer();

    /** Start and stop the server */
    void run();
    void stop();

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_SIMPLE_NETWORK_SERVER_H
