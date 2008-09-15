/*
 * opencog/server/ServerSocket.h
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

#ifndef _OPENCOG_SERVER_SOCKET_H
#define _OPENCOG_SERVER_SOCKET_H

#include <string>

#include <Sockets/TcpSocket.h>
#include <Sockets/ISocketHandler.h>

#include <opencog/server/SimpleNetworkServer.h>
#include <opencog/server/CallBackInterface.h>

namespace opencog
{


class ServerSocket : public TcpSocket
{
private:

    // Used to call-back
    static SimpleNetworkServer *master;

    // Buffer to store raw data streaming in over a socket. The data
    // needs to be buffered because the XML parser wasn't designed to
    // run in streaming mode, it can only batch the requests :-(
    std::string buffer;
    bool in_raw_mode;
    bool have_raw_eot;

    // If the client closes the socket before the command
    // has been processed, then an instance of this class
    // will no longer exist. Yet, the callback itself must
    // still exist, so that it can be called. The CBI class
    // exists entirely so that the callback remains instantiated
    // even if the socket is closed.
    class CBI : public CallBackInterface
    {
    private:
        ServerSocket *sock;
        pthread_mutex_t sock_lock;
        int use_count;
        pthread_mutex_t use_count_lock;
    public:
        CBI(ServerSocket *);
        void Close(void);
        int AtomicInc(int inc);
        void callBack(const std::string &message);
    };
    CBI *cb;

public:

    ~ServerSocket();
    ServerSocket(ISocketHandler &handler);

    static void setMaster(SimpleNetworkServer *master);
    void OnAccept();
    void OnDisconnect();
    void OnLine(const std::string&);
    void OnRawData(const char *, size_t);
}; // class

}  // namespace

#endif // _OPENCOG_SERVER_SOCKET_H
