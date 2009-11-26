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

#ifndef _OPENCOG_SOCKET_HOLDER_H
#define _OPENCOG_SOCKET_HOLDER_H

#include <pthread.h>

#define REPLACE_CSOCKETS_BY_ASIO
#ifndef REPLACE_CSOCKETS_BY_ASIO
#include <Sockets/TcpSocket.h>
#endif
#include <opencog/shell/GenericShell.h>

namespace opencog
{

    class ServerSocket;
/**
 * This class is a "holder" or "handle" for the Alhem CSocket library
 * sockets.  It's only reason for existence is to work around a
 * "feature" in the Alhem CSocket design: when a remote client closes
 * it's end of the socket, then the CSockets library will call the
 * destructor on the TcpSocket class. Thus, any code that holds a
 * pointer to a TcpSocket instance (such as class Request, which
 * interacts with ConsoleSocket), and was hoping to send data, would
 * end up calling methods on a non-existant object .. i.e. would be
 * dereferencing a garbage pointer, and crashing.
 *
 * Thus, generically, it is not safe for just about any part of OpenCog
 * to try to access an Alhem socket, since it might not actually be
 * there at the time of access.  This class provides a safe "wrapper"
 * around the Alhem Sockets, and continues to live on even if the socket
 * is gone.  Thus, all OpenCog code should access socket functions only
 * through this wrapper.
 *
 * Any user of this class may hold a pointer to an instance of it; but,
 * if it does so, it must increment its use-count for as long as it
 * holds on. When done with this instance, the user should decrement the
 * use count. After decrementing the use count, the instance may vanish,
 * and so should not be used again.
 *
 * (This class is one example where garbage collection inside of OpenCog
 * would make life simpler and safer.)
 */

class SocketHolder
{
    private:
#ifdef REPLACE_CSOCKETS_BY_ASIO
        ServerSocket *_sock;
#else
        TcpSocket *_sock;
#endif
        pthread_mutex_t sock_lock;
        int use_count;
        pthread_mutex_t use_count_lock;

        void do_rpc_complete(void);

    public:
        SocketHolder(void);
        ~SocketHolder();

#ifdef REPLACE_CSOCKETS_BY_ASIO
        void setSocket(ServerSocket *s);
#else
        void setSocket(TcpSocket *s);
#endif
        int AtomicInc(int inc);

        // Misc OpenCog socket things.
        std::string mimeType(void);
        void SetShell(GenericShell *);

        // The following are "wrappers" around TcpSocket calls
        void send(const std::string& msg);
        void SetCloseAndDelete(void);
        void SetLineProtocol(bool);

}; // class

} //namespace
 
#endif /* _OPENCOG_SOCKET_HOLDER_H */
