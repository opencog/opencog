/*
 * opencog/cogserver/server/ServerSocket.h
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

#ifndef _OPENCOG_SERVER_SOCKET_H
#define _OPENCOG_SERVER_SOCKET_H

#include <boost/asio.hpp>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

/**
 * This class defines the minimal set of methods a server socket must
 * have to handle the primary interface of the cogserver.
 *
 * Each ServerSocket supports a client that connects to the cog server.
 */
class ServerSocket
{
private:
    boost::asio::ip::tcp::socket _socket;
    bool _lineProtocol;
    bool _closed;

protected:
    /**
     * Connection callback: called whenever a new connection arrives
     */
    virtual void OnConnection(void) = 0;

    /**
     * OnLine callback: called when in LineProtocol mode and a new
     * line is received from the client.
     */
    virtual void OnLine (const std::string&) = 0;

    /**
     * OnRawData callback: called when LineProtocol is disabled and
     * new data is received from the client.
     */
    virtual void OnRawData (const char*, size_t) = 0;

public:
    ServerSocket(boost::asio::io_service&);
    virtual ~ServerSocket();

    static void handle_connection(ServerSocket*);
    boost::asio::ip::tcp::socket& getSocket(void);

    /**
     * Sends data to the client
     */
    void Send(const std::string&);

    /**
     * Close this socket
     */
    void SetCloseAndDelete(void);

    /**
     *Set LineProtocol mode for this socket
     */
    void SetLineProtocol(bool);

    /**
     * Check if this socket is in LineProtocol mode
     */
    bool LineProtocol(void);

    /**
     * Check if this socket was closed
     */
    bool isClosed();

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_SERVER_SOCKET_H
