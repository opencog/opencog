/**
 * ServerSocket.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jan 28 20:58:48 BRT 2007
 */

#ifndef SERVERSOCKET_H
#define SERVERSOCKET_H

#include <string>

#include <TcpSocket.h>
#include <ISocketHandler.h>
#include "SimpleNetworkServer.h"
#include "CallBackInterface.h"

namespace opencog {


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
	         public:
                CBI(ServerSocket *);
                void Close(void);
                void callBack(const std::string &message);
        };
        CBI *cb;

    public:

        ~ServerSocket();
        ServerSocket(ISocketHandler &handler);

        static void setMaster(SimpleNetworkServer *master);
        void OnDisconnect();
        void OnLine(const std::string&);
        void OnRawData(const char *, size_t);
}; // class
}  // namespace

#endif
