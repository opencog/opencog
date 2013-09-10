/*
 * opencog/server/NetworkServer.h
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

#include <opencog/server/SocketListener.h>

#include <opencog/util/Logger.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */


/**
 * This class implements the entity responsible for managing all the opencog
 * network servers. It uses alhem's sockets library
 * (http://www.alhem.net/Sockets/) to handle the low level network sockets.
 *
 * The network server runs on its own thread (thus freeing the cogserver's main
 * loop to deal with requests and agents only). And it may be enabled/disabled
 * at will so that a cogserver may run in networkless mode if desired.
 * 
 * The network server supports multiple server sockets. Client applications
 * should use the 'addListener' functor and the 'removeListener' method to
 * add/remove custom server sockets. Currently, the network server doesn't
 * support selecting the network interface that the server socket will bind to
 * (every server sockets binds to 0.0.0.0, i.e., all interfaces). Thus,
 * server sockets are identified/selected by the port they bind to.
 *
 * Another limitation of the network server is that is adds a delay to certain
 * network operations. Due to the way the alhem's sockets library async loop is
 * built around 'select', some operations (such as forwarding a client socket to
 * a separate thread) take a few mili-seconds to complete. This delay is
 * proportional to the timeout T supplied to the sockets library by the network
 * server. Currently we use T == * 0.2 seconds, which doesn't seem to increase
 * latency too much. The downside is that the opencog server inhibits proper
 * power manager by the OS, as it wakes up the processor at a relatively high
 * frequency.
 */
class NetworkServer
{

protected:

    bool _started;
    bool _running;
    boost::asio::io_service io_service;
    std::vector<SocketPort*> _listeners;
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
    template<class _Socket>
    bool addListener(const unsigned int port) {
        logger().debug("adding listener to port %d", port);
        SocketListener<_Socket>* sl = new SocketListener<_Socket>(io_service, port);
        //TODO: Error handling (what if bind does not work?)
        _listeners.push_back(sl);
        printf("Listening on port %d\n", port);
        return true;
    }

    /** Closes the listener socket bound to port 'port'. Returns 'true'
     * if successful and 'false' otherwise.
     */
    bool removeListener(const unsigned short port);

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_SIMPLE_NETWORK_SERVER_H
