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
    _acceptor(_io_service,
        boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
{
    logger().debug("[NetworkServer] constructor");

    // XXX FIXME ... this is leaking memory -- theres no dtor for
    // this socket !?  Or does tcp::acceptor magically release it?
    ConsoleSocket* ss = new ConsoleSocket(_io_service);
    _acceptor.async_accept(ss->getSocket(),
         boost::bind(&NetworkServer::handle_accept,
         this, ss,
         boost::asio::placeholders::error));

    _thread = new std::thread(&NetworkServer::run, this);
    printf("Listening on port %d\n", port);
    _running = true;
}

NetworkServer::~NetworkServer()
{
    logger().debug("[NetworkServer] enter destructor");

    stop();

    logger().debug("[NetworkServer] all threads joined, exit destructor");
}

void NetworkServer::stop()
{
    _running = false;
    _io_service.stop();

    _thread->join();
    delete _thread;
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

void NetworkServer::handle_accept(ConsoleSocket* ss, const
                                  boost::system::error_code& error)
{
    logger().debug("NetworkServer::handle_accept() started");
    if (!error)
    {
        ss->start();
        ConsoleSocket* nss = new ConsoleSocket(_io_service);
        // _accepted_sockets.push_back(nss);
        _acceptor.async_accept(nss->getSocket(),
              boost::bind(&NetworkServer::handle_accept,
              this, nss,
              boost::asio::placeholders::error));
    }
    else
    {
        // TODO: add more data to the error log (using the passed
        // boost::system::error_code)
        logger().error("NetworkServer::handle_accept(): Accept error");
        delete ss;
    }
    logger().debug("NetworkServer::handle_accept() ended");
}
