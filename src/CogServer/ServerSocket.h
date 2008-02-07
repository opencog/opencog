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

class ServerSocket : public TcpSocket, CallBackInterface {

    private:

        // Used to call-back
        static SimpleNetworkServer *master;
        bool in_raw_mode;

        // Buffer to store raw data streaming in over a socket. The data
        // needs to be buffered because the XML parser wasn't designed to 
        // run in streaming mode, it can only batch the requests :-(
        std::string buffer;
    public:

        ~ServerSocket();
        ServerSocket(ISocketHandler &handler);

        static void setMaster(SimpleNetworkServer *master);
        void OnDisconnect();
        void OnLine(const std::string&);
        void OnRawData(const char *, size_t);
        void callBack(const std::string &message);

}; // class
}  // namespace

#endif
