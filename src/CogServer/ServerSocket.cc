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

ServerSocket::~ServerSocket()
{
    cb->Close();
}

ServerSocket::ServerSocket(ISocketHandler &handler):TcpSocket(handler)
{
    SetLineProtocol();
    in_raw_mode = false;
    buffer = "";
    cb = new CBI(this);
}

void ServerSocket::setMaster(SimpleNetworkServer *m) {
    master = m;
}

void ServerSocket::OnDisconnect()
{
    if (!in_raw_mode) return;

    cb->AtomicInc(1);
    master->processCommandLine(cb, buffer);
    SetLineProtocol(true);
    buffer = "";
    in_raw_mode = false;
}

void ServerSocket::OnLine(const std::string& line)
{
    if (line == "data")
    {
        // Disable line protocol; we are expecting a stream
        // of bytes from now on, until socket closure.
        // The OnRawData() method will be called from here 
        // until a ctrl-D or socket close.
        SetLineProtocol(false);
        in_raw_mode = true;
        buffer = "data\n";
    }
    else
    {
        cb->AtomicInc(1);
        master->processCommandLine(cb, line);
    }
}

/**
 * Buffer up incoming raw data until an end-of-transmission (EOT)
 * is received. After the EOT, dispatch the buffered data as a single
 * command request.
 *
 * Note that there is just one ServerSocket per client, so the buffer
 * is not shared.
 *
 * XXX the data buffer is currently just a string buffer, so any
 * null char in the data stream will terminate the string. So
 * this method isn't really for "raw" data containing null chars.
 * This should be fixed someday, as needed.
 */
void ServerSocket::OnRawData(const char * buf, size_t len)
{
    // Close on ctrl-D aka ASCII EOT
    if ((len == 1) && (buf[0] == 0x4)) {
        OnDisconnect();
        return;
    }
    buffer += buf;
}

ServerSocket::CBI::CBI(ServerSocket *s)
{
    pthread_mutex_init(&sock_lock, NULL);
    sock = s;
    pthread_mutex_init(&use_count_lock, NULL);
    use_count = 1;
}

int ServerSocket::CBI::AtomicInc(int inc)
{
    int uc;
    pthread_mutex_lock(&use_count_lock);
    use_count += inc;
    uc = use_count;
    pthread_mutex_unlock(&use_count_lock);
    return uc;
}

void ServerSocket::CBI::Close(void)
{
    pthread_mutex_lock(&sock_lock);
    sock = NULL;
    int cnt = AtomicInc(-1);
    pthread_mutex_unlock(&sock_lock);

    // There will be one callback for each request that 
    // was generated on this socket. When all of these
    // have been processed, then self-delete, as no one
    // else will.
    if (cnt <= 0) delete this;
}

void ServerSocket::CBI::callBack(const std::string &message)
{
    // If the socket is closed, then we can't send anything at all.
    // Note that the sock is typically closed in a distinct thread,
    // which can race, and so lock to make sure that sock != NULL.
    pthread_mutex_lock(&sock_lock);
    if(sock) {
        std::istringstream stream(message.c_str());
        std::string line;

        while (getline(stream, line)) {
            sock->Send(line + "\n");
        }
    }
    pthread_mutex_unlock(&sock_lock);

    // There will be one callback for each request that 
    // was generated on this socket. When all of these
    // have been processed, then self-delete, as no one
    // else will.
    int cnt = AtomicInc(-1);
    if (cnt <= 0) delete this;
}
