/*
 * opencog/cogserver/server/NetworkServer.cc
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
    : _started(false), _running(false), _listener(NULL), _thread(0)
{
    logger().debug("[NetworkServer] constructor");
}

NetworkServer::~NetworkServer()
{
    logger().debug("[NetworkServer] enter destructor");

    delete _listener;
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
    _io_service.stop();
    if (_thread != 0)
        pthread_join(_thread, NULL);
} 

void NetworkServer::run()
{
    logger().debug("[NetworkServer] run");
    while (_running) {
        try {
            _io_service.run();
        } catch (boost::system::system_error& e) {
            logger().error("Error in boost::asio io_service::run() => %s", e.what());
        }
        usleep(50000); // avoids busy wait
    }
    logger().debug("[NetworkServer] end of run");
    _started = false;
}


void NetworkServer::addListener(const unsigned int port)
{
    if (_listener)
    {
        printf("Only one port is allowed\n");
        exit(1);
    }
    _listener = new SocketListener(_io_service, port);
    printf("Listening on port %d\n", port);
}
