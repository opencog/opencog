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

void ServerSocket::OnLine(const std::string& line)
{
    if (line == "data")
    {
        // Disable line protocol; we are expecting a stream
        // of bytes from now on, until socket closure.
        // The OnRawData() method will be called from here on out.
        SetLineProtocol(false);
    }
    else
    {
        master->processCommandLine(this, line);
    }
}

void ServerSocket::OnRawData(const char * buf, size_t len)
{
    // Close on ctrl-D aka ASCII EOT
    if ((len == 1) && (buf[0] == 0x4)) {
        Close();
    } else {
        master->processData(this, buf, len);
    }
}

void ServerSocket::callBack(const std::string &message) {

    std::istringstream stream(message.c_str());
    std::string line;

    while (getline(stream, line)) {
        Send(line + "\n");
    }
}
