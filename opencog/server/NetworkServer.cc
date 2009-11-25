/*
 * opencog/server/NetworkServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
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

#include "NetworkServer.h"

#include <algorithm>
#include <tr1/memory>
#include <tr1/functional>

#include <Sockets/SocketHandler.h>
#include <Sockets/ListenSocket.h>

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/platform.h>

using namespace opencog;

NetworkServer::NetworkServer()
    : _running(false), _thread(0)
#ifndef REPLACE_CSOCKETS_BY_ASIO
      , _shandler(_mutex)
#endif
{
    logger().debug("[NetworkServer] constructor");
}

NetworkServer::~NetworkServer()
{
    logger().debug("[NetworkServer] destructor");
    if (_thread != 0)
        pthread_join(_thread, NULL);
    logger().debug("[NetworkServer] all threads joined");
}

extern "C" {
static void* _opencog_run_wrapper(void* arg)
{
    logger().debug("[NetworkServer] run wrapper");
    NetworkServer* ns = (NetworkServer*) arg;
    ns->run();
    return NULL;
}
}

void NetworkServer::start()
{
    if (_running) {
        logger().warn("server has already started");
        return;
    }

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    // create thread
    _running = true;
    int rc = pthread_create(&_thread, &attr, _opencog_run_wrapper, this);
    if (rc != 0) {
        logger().error("Unable to start network server thread: %d", rc);
        _running = false;
    }
}

void NetworkServer::stop()
{
    logger().debug("[NetworkServer] stop");
    _running = false;
#ifdef REPLACE_CSOCKETS_BY_ASIO
    io_service.stop();
#endif
}

void NetworkServer::run()
{
    logger().debug("[NetworkServer] run");
    while (_running) {
#ifdef REPLACE_CSOCKETS_BY_ASIO
        try {
            io_service.run();
        } catch (boost::system::system_error& e) {
            logger().error("Error in boost::asio io_service::run() => %s", e.what());
        }
        usleep(500000); // avoids busy wait
#else
        // we should use a larger value for the select timeout (< 1s prevents
        // the server from saving energy because we wake up too often)
        // however, the current implementations of the SocketHandler class
        // only checks for the detach/close/connect events *after* the
        // select timeout expires; so if we use a larger timeout some of the
        // server events will be issued after a long delay
        _shandler.Select(0, 200000);
#endif
    }
    logger().debug("[NetworkServer] end of run");
}

namespace opencog {
#ifdef REPLACE_CSOCKETS_BY_ASIO
struct equal_to_port : public std::binary_function<const SocketPort*, const unsigned short &, bool>
{
    bool operator()(const SocketPort* sock, const unsigned short &port) {
        SocketPort* s = const_cast<SocketPort*>(sock);
        return ((s->getPort()) == port);
#else
struct equal_to_port : public std::binary_function<const Socket*, const unsigned short &, bool>
{
    bool operator()(const Socket* sock, const unsigned short &port) {
        Socket* s = const_cast<Socket*>(sock);
        return ((s->GetPort()) == port);
#endif
    }
};
}

bool NetworkServer::removeListener(const unsigned short port)
{
    logger().debug("[NetworkServer] removing listener bound to port %d", port);
#ifdef REPLACE_CSOCKETS_BY_ASIO
    std::vector<SocketPort*>::iterator l = 
#else
    std::vector<Socket*>::iterator l = 
#endif
        std::find_if(_listeners.begin(), _listeners.end(),
                     std::tr1::bind(equal_to_port(), std::tr1::placeholders::_1, port));
    if (l == _listeners.end()) {
        logger().warn("unable to remove listener from port %d", port);
        return false;
    }
    delete *l;
    _listeners.erase(l);
    return true;
}
