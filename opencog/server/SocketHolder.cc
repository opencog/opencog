/*
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
 *
 * Copyright (C) 2008 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
 */

#include "ConsoleSocket.h"
#include "IHasMimeType.h"
#include "IRPCSocket.h"
#include "SocketHolder.h"

#include <Sockets/Lock.h>
#include <Sockets/ISocketHandler.h>
#include <Sockets/Socket.h>

#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

using namespace opencog;

SocketHolder::SocketHolder(void)
{
    _sock = NULL;
    pthread_mutex_init(&sock_lock, NULL);
    pthread_mutex_init(&use_count_lock, NULL);
    use_count = 1;
}

SocketHolder::~SocketHolder()
{
}

#ifdef REPLACE_CSOCKETS_BY_ASIO
void SocketHolder::setSocket (ServerSocket *s)
#else
void SocketHolder::setSocket (TcpSocket *s)
#endif
{
    pthread_mutex_lock(&sock_lock);
    _sock = s;
    pthread_mutex_unlock(&sock_lock);
}

void SocketHolder::do_rpc_complete(void)
{
    pthread_mutex_lock(&sock_lock);
    if (_sock) {
       IRPCSocket* _rpcsock = dynamic_cast<IRPCSocket*>(_sock);
       if (_rpcsock) {
           _rpcsock->OnRequestComplete();
       }
    }
    pthread_mutex_unlock(&sock_lock);
}

int SocketHolder::AtomicInc(int inc)
{
    if (0 > inc) do_rpc_complete();

    int uc;
    pthread_mutex_lock(&use_count_lock);
    use_count += inc;
    uc = use_count;
    pthread_mutex_unlock(&use_count_lock);


    // When all users of this socket have finished using it,
    // self-delete.  No one else will :-)
    if (uc <= 0) delete this;

    return uc;
}

void SocketHolder::send(const std::string& msg)
{
    logger().debug("[SocketHolder] send\n");
    pthread_mutex_lock(&sock_lock);
    if (_sock) {
#ifndef REPLACE_CSOCKETS_BY_ASIO
        Lock l(_sock->MasterHandler().GetMutex());
#endif
        _sock->Send(msg);
    }
    pthread_mutex_unlock(&sock_lock);
}

void SocketHolder::SetCloseAndDelete(void)
{
    pthread_mutex_lock(&sock_lock);
    if (_sock) {
        _sock->SetCloseAndDelete();
    }
    pthread_mutex_unlock(&sock_lock);
}

void SocketHolder::SetLineProtocol(bool b)
{
    pthread_mutex_lock(&sock_lock);
    if (_sock) {
        _sock->SetLineProtocol(b);
    }
    pthread_mutex_unlock(&sock_lock);
}

std::string SocketHolder::mimeType(void)
{
    std::string rc = "";

    pthread_mutex_lock(&sock_lock);
    if (_sock) {
        IHasMimeType* ihmt = dynamic_cast<IHasMimeType*>(_sock);
        if (ihmt == NULL)
        {
            pthread_mutex_unlock(&sock_lock);
            throw RuntimeException(TRACE_INFO,
                "invalid socket: it does not have a mime-type.");
        }
        rc = ihmt->mimeType();
    }
    pthread_mutex_unlock(&sock_lock);

    return rc;
}

void SocketHolder::SetShell(GenericShell *gs)
{
    pthread_mutex_lock(&sock_lock);
    if (_sock) {
       ConsoleSocket *cs = dynamic_cast<ConsoleSocket *>(_sock);
       if (cs) cs->SetShell(gs);
    }
    pthread_mutex_unlock(&sock_lock);
}
