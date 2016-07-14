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

#include <pthread.h>
#include <boost/asio.hpp>

#include <opencog/cogserver/server/SocketListener.h>
#include <opencog/cogserver/server/ConsoleSocket.h>

#include <opencog/util/Logger.h>

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
 * should use the 'addListener' functor to add custom server sockets.
 * Currently, the network server doesn't
 * support selecting the network interface that the server socket will bind to
 * (every server sockets binds to 0.0.0.0, i.e., all interfaces). Thus,
 * server sockets are identified/selected by the port they bind to.
 */
class NetworkServer
{

protected:

    bool _started;
    bool _running;
    boost::asio::io_service _io_service;
    SocketListener<ConsoleSocket>* _listener;
    pthread_t _thread;

public:

    /** NetworkServer's contructor. Initializes the threading control
     *  class members. */
    NetworkServer();

    /** NetworkServer's destructor. */
    virtual ~NetworkServer();

    /** Starts the NetworkServer by creating a new pthread and
     * (indirectly) calling the method 'run'
     */
    virtual void start();

    /** Stops the NetworkServer by flipping the flag that controls the
     * main loop.
     */
    virtual void stop();

    /** The network server's thread main method. It should not be
     * called by external classes and is only declared as public
     * because we use a 'extern "C"' wrapper when calling 
     * pthread_create (many C++ 'best practices' guides state that
     * directly using the C++ 'run' is unsafe)
     */
    void run();

    /** Instantiates a listener socket (i.e. server socket) of class
     * '_Socket' and binds it to port 'port'. Returns 'true' if
     * successful and 'false' otherwise.
     */
    void addListener(const unsigned int port);
}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_SIMPLE_NETWORK_SERVER_H
