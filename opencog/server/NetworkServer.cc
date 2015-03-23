/*
 * opencog/server/NetworkServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <boost/bind.hpp>

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/platform.h>

using namespace opencog;

NetworkServer::NetworkServer()
    : _started(false), _running(false), _thread(0)
{
    logger().debug("[NetworkServer] constructor");
}

NetworkServer::~NetworkServer()
{
    logger().debug("[NetworkServer] enter destructor");
    if (_thread != 0)
        pthread_join(_thread, NULL);

    for (SocketPort* sp : _listeners) delete sp;
    logger().debug("[NetworkServer] all threads joined, exit destructor");
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
    int rc = pthread_create(&_thread, &attr, _opencog_run_wrapper, this);
    if (rc != 0) {
        logger().error("Unable to start network server thread: %d", rc);
        _running = false;
        _started = false;
        return;
    }
    _started = true;
    _running = true;
}

void NetworkServer::stop()
{
    logger().debug("[NetworkServer] stop");
    _running = false;
    io_service.stop();
    while (_started) { usleep(10000); }
} 

void NetworkServer::run()
{
    logger().debug("[NetworkServer] run");
    while (_running) {
        try {
            io_service.run();
        } catch (boost::system::system_error& e) {
            logger().error("Error in boost::asio io_service::run() => %s", e.what());
        }
        usleep(50000); // avoids busy wait
    }
    logger().debug("[NetworkServer] end of run");
    _started = false;
}

namespace opencog {
struct equal_to_port : public std::binary_function<const SocketPort*, const unsigned short &, bool>
{
    bool operator()(const SocketPort* sock, const unsigned short &port) {
        SocketPort* s = const_cast<SocketPort*>(sock);
        return ((s->getPort()) == port);
    }
};
}

bool NetworkServer::removeListener(const unsigned short port)
{
    logger().debug("[NetworkServer] removing listener bound to port %d", port);
    std::vector<SocketPort*>::iterator l = 
        std::find_if(_listeners.begin(), _listeners.end(),
                     boost::bind(equal_to_port(), _1, port));
    if (l == _listeners.end()) {
        logger().warn("unable to remove listener from port %d", port);
        return false;
    }
    SocketPort* sp = *l;
    _listeners.erase(l);
    delete sp;
    return true;
}
