/*
 * opencog/socket/GenericSocket.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_GENERIC_SOCKET_H
#define _OPENCOG_GENERIC_SOCKET_H

#include <string>
#include <sstream>
#include <tr1/memory>

#include <Sockets/TcpSocket.h>
#include <Sockets/ISocketHandler.h>

#include <opencog/server/IRPCSocket.h>

namespace opencog
{

class GenericShell;

class GenericSocket : public TcpSocket
{

protected:

    GenericShell* _shell;

public:

    GenericSocket(ISocketHandler &handler);
    virtual ~GenericSocket();

    void DeclareShell(GenericShell*);

    virtual void OnAccept          (void);
    virtual void OnDetached        (void);
    virtual void OnLine            (const std::string& line);

}; // class

}  // namespace

#endif // _OPENCOG_GENERIC_SOCKET_H
