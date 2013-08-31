/*
 * opencog/server/SocketListener.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Luigi <welter@vettalabs.com>
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

#ifndef _OPENCOG_SOCKET_LISTENER_H
#define _OPENCOG_SOCKET_LISTENER_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <opencog/server/SocketPort.h>
#include <opencog/util/Logger.h>

using boost::asio::ip::tcp;

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

/**
 * This class defines a socket listener for a given port number 
 */
template <class _Socket>
class SocketListener : public SocketPort
{

private:

    boost::asio::io_service& io_service;
    tcp::acceptor acceptor;

public:

    SocketListener(boost::asio::io_service& _io_service, int _port) :
        SocketPort(_port),
        io_service(_io_service),
        acceptor(_io_service, tcp::endpoint(tcp::v4(), _port))
    {
        logger().debug("SocketListener::SocketListener() started");
        logger().debug("Acceptor listening.");
        _Socket* ss = new _Socket(io_service);
        acceptor.async_accept(ss->getSocket(),
                              boost::bind(&SocketListener::handle_accept, this, ss,
                              boost::asio::placeholders::error));
        logger().debug("SocketListener::SocketListener() ended");
    }


    void handle_accept(_Socket* ss, const boost::system::error_code& error)
    {
        logger().debug("SocketListener::handle_accept() started");
        if (!error)
        {
            ss->start();
            ss = new _Socket(io_service);
            acceptor.async_accept(ss->getSocket(),
                                  boost::bind(&SocketListener::handle_accept, this, ss,
                                  boost::asio::placeholders::error));
        }
        else
        {
            // TODO: add more data to the error log (using the passed
            // boost::system::error_code)
            logger().error("SocketListener::handle_accept(): Accept error");
            delete ss;
        }
        logger().debug("SocketListener::handle_accept() ended");
    }


}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_SOCKET_LISTENER_H


