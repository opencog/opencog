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

#include <Sockets/TcpSocket.h>
#include <opencog/shell/GenericShell.h>

namespace opencog
{

/**
 * This class is a "holder" or "handle" for the Alhem CSocket library
 * sockets.  It's only reason for existence is to work around a
 * "feature" in the Alhem CSocket design: when a remote client closes
 * it's end of the socket, then the CSockets library will call the
 * destructor on the TcpSocket class. Thus, any code that holds a
 * pointer to a TcpSocket instance (such as class Request, which
 * interacts with ConsoleSocket), and was hoping to send data, would
 * end up calling methods on a non-existant object .. i.e.  * would be
 * dereferencing a garbage pointer, and crashing.
 *
 * This class manages that pointer, and continues to live after the
 * socket has disappeared. It accomplishes this by means of reference-
 * counting: it maintains a use count, and self-destructs when that
 * use-count has gone to zero. (This is one example where garbage
 * collection inside of OpenCog would make life simpler and safer.)
 */

class SocketHolder
{
    private:
        TcpSocket *_sock;
        pthread_mutex_t sock_lock;
        int use_count;
        pthread_mutex_t use_count_lock;

        void do_rpc_complete(void);

    public:
        SocketHolder(void);
        ~SocketHolder();

        void setSocket(TcpSocket *s);
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
