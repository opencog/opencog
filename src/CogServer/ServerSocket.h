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

    public:

        ~ServerSocket();
        ServerSocket(ISocketHandler &handler);

        static void setMaster(SimpleNetworkServer *master);
        void OnLine(const std::string&);
        void OnRawData(const char *, size_t);
        void callBack(const std::string &message);

}; // class
}  // namespace

#endif
