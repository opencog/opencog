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

#include <Sockets/Mutex.h>
#include <Sockets/Lock.h>
#include <Sockets/SocketHandler.h>
#include <Sockets/ListenSocket.h>

#include <opencog/server/NetworkServer.h>
#include <opencog/util/Logger.h>

namespace opencog
{

class NetworkServer
{

protected:

    bool _started;
    bool _running;
    Mutex _mutex;
    SocketHandler _shandler;
    std::vector<Socket*> _listeners;
    pthread_t _thread;

public:

    NetworkServer();
    virtual ~NetworkServer();

    virtual void start();
    virtual void stop();
    virtual void run();

    template<class _Socket>
    bool addListener(const unsigned int port) {
        logger().debug("adding listener to socket %d", port);
        ListenSocket<_Socket>* lsocket = new ListenSocket<_Socket>(_shandler);
        try {
            // we throw an exception ourselves because csockets may
            // be compiled with exceptions disabled
            if (lsocket->Bind(port)) throw new Exception("bind error");
            logger().info("server listening on port %d.", port);
        } catch (Exception) {
            logger().error("unable to bind to port %d. Aborting.", port);
            return false;
        }
        _shandler.Add(lsocket);
        _listeners.push_back(lsocket);
        return true;
    }

    bool removeListener(const unsigned short port);

}; // class

}  // namespace

#endif // _OPENCOG_SIMPLE_NETWORK_SERVER_H
