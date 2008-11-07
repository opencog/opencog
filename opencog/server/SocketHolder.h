
#ifndef _OPENCOG_SOCKET_HOLDER_H
#define _OPENCOG_SOCKET_HOLDER_H

#include <Sockets/TcpSocket.h>

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
 * socket has disappeared. It accomplishes this by means of
 * reference-counting: it maintains a use count, and self-destructs when
 * that use-count has gone to zero.
 */

class SocketHolder
{
    private:
        TcpSocket *_sock;

    public:
        SocketHolder(void);
        ~SocketHolder();

        void setSocket(TcpSocket *s);
        TcpSocket *getSocket(void);
        void send(const std::string& msg) const;

}; // class

} //namespace
 
#endif /* _OPENCOG_SOCKET_HOLDER_H */
