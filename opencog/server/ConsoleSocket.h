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
#include <sstream>
#include <tr1/memory>

#include <Sockets/TcpSocket.h>
#include <Sockets/ISocketHandler.h>

#include <opencog/server/IHasMimeType.h>
#include <opencog/server/IRPCSocket.h>

namespace opencog
{

class Request;

class ConsoleSocket : public TcpSocket,
                      public IHasMimeType,
                      public IRPCSocket
{

protected:

    Request* _request;
    std::stringstream _buffer;

public:
    static const int RESPONSE_TRIGGER = 0;

    ~ConsoleSocket();
    ConsoleSocket(ISocketHandler &handler);

    void OnAccept          (void);
    void OnDetached        (void);
    void OnLine            (const std::string& line);
    void OnRawData         (const char * buf, size_t len);
    void OnRequestComplete ();

}; // class

}  // namespace

#endif // _OPENCOG_SERVER_SOCKET_H
