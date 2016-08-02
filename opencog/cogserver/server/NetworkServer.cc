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

NetworkServer::NetworkServer(unsigned short port) :
    _running(true),
    _port(port),
    _acceptor(_io_service,
        boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
{
    logger().debug("[NetworkServer] constructor");

    _service_thread = new std::thread(&NetworkServer::run, this);
}

NetworkServer::~NetworkServer()
{
    logger().debug("[NetworkServer] enter destructor");

    stop();

    logger().debug("[NetworkServer] all threads joined, exit destructor");
}

void NetworkServer::stop()
{
    if (not _running) return;
    _running = false;

    boost::system::error_code ec;
    _acceptor.cancel(ec);
    _io_service.stop();

    _listener_thread->join();
    _listener_thread = nullptr;
    _service_thread->join();
    _service_thread = nullptr;
}

void NetworkServer::listen()
{
    printf("Listening on port %d\n", _port);
    while (_running)
    {
        // The call to _acceptor.accept() will block this thread until
        // a network connection is made. Thus, we defer the creation
        // of the connection handler thread until after accept()
        // returns.  However, the boost design violates RAII principles,
        // so instead, what we do is to hand off the socket created here,
        // to the ServerSocket class, which will delete it, when the
        // connection socket closes (just before the connection handler
        // thread exits).  That is why there is no delete of the *ss
        // below, and that is why there is the weird self-delete at the
        // end of ServerSocket::handle_connection().
        boost::asio::ip::tcp::socket* sock = new boost::asio::ip::tcp::socket(_io_service);
        _acceptor.accept(*sock);

        ConsoleSocket* ss = new ConsoleSocket();
        ss->set_connection(sock);
        std::thread(&ConsoleSocket::handle_connection, ss).detach();
    }
}

void NetworkServer::run()
{
    _listener_thread = new std::thread(&NetworkServer::listen, this);

    while (_running)
    {
        try {
            _io_service.run();
        } catch (boost::system::system_error& e) {
            logger().error("Error in boost::asio io_service::run() => %s", e.what());
        }
        usleep(50000); // avoids busy wait
    }
}
