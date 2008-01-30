/**
 * ServerSocket.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jan 28 20:58:48 BRT 2007
 */

#include "ServerSocket.h"
#include <string> 
#include <sstream> 

using namespace opencog;

SimpleNetworkServer *ServerSocket::master = NULL;

ServerSocket::~ServerSocket() {
}

ServerSocket::ServerSocket(ISocketHandler &handler):TcpSocket(handler) {
    SetLineProtocol();
}

void ServerSocket::setMaster(SimpleNetworkServer *m) {
    master = m;
}

void ServerSocket::OnLine(const std::string& line) {
    master->processCommandLine(this, line);
}

void ServerSocket::callBack(const std::string &message) {

    std::istringstream stream(message.c_str());
    std::string line;

    while (getline(stream, line)) {
        Send(line + "\n");
    }
}
