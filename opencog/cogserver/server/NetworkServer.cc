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

#include <opencog/util/Logger.h>

using namespace opencog;

NetworkServer::NetworkServer()
    : _running(false), _thread(NULL), _listener(NULL)
{
    logger().debug("[NetworkServer] constructor");
}

NetworkServer::~NetworkServer()
{
    logger().debug("[NetworkServer] enter destructor");

    stop();

    logger().debug("[NetworkServer] all threads joined, exit destructor");
}

void NetworkServer::start(unsigned short port)
{
    if (_running)
    {
        printf("Only one port is allowed\n");
        exit(1);
    }

    _running = true;
    _listener = new SocketListener(_io_service, port);
    _thread = new std::thread(&NetworkServer::run, this);
    printf("Listening on port %d\n", port);
}

void NetworkServer::stop()
{
    if (not _running) return;

    logger().debug("[NetworkServer] stop");
    _running = false;
    _io_service.stop();
    if (_thread)
    {
        _thread->join();
        delete _thread;
        _thread = NULL;

        delete _listener;
        _listener = NULL;
    }
}

void NetworkServer::run()
{
    while (_running) {
        try {
            _io_service.run();
        } catch (boost::system::system_error& e) {
            logger().error("Error in boost::asio io_service::run() => %s", e.what());
        }
        usleep(50000); // avoids busy wait
    }
}
