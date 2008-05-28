/*
 * src/CogServer/ServerSocket.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
 *
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
    have_raw_eot = false;
    buffer = "";
    cb = new CBI(this);
}

void ServerSocket::setMaster(SimpleNetworkServer *m) {
    master = m;
}

void ServerSocket::OnAccept()
{
    Send(master->getCommandPrompt());
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
    size_t i;
    size_t istart = 0;
    while (istart<len)
    {
        // Search for ctrl-D aka ASCII EOT, followed by newline
        for (i=istart; i<len-1; i++)
        {
            if ((0x4 == buf[i]) && ((0xd == buf[i+1]) || (0xa == buf[i+1]))) break;
            if (have_raw_eot && ((0xd == buf[i]) || (0xa == buf[i]))) {
                have_raw_eot = false;
                break;
            }
        }
        if (i == len-1)
        {
            if(0x4 == buf[i]) {
                have_raw_eot = true;
                len --;  // Do not copy the ctrl-D
            }
            // No ctrl-D found, append the string, and go home
            buffer.append (&buf[istart], len-istart);
            return;
        }

        // Found a ctrl-D, dispatch the thing.
        int iend = i;
        buffer.append (&buf[istart], iend-istart);
        cb->AtomicInc(1);
        master->processCommandLine(cb, buffer);
        buffer = "";
        istart = iend;

        // Skip over the ctrl-D and any newlines/carriage returns.
        while ((0xa == buf[istart]) || 
               (0xd == buf[istart]) || 
               (0x4 == buf[istart])) istart ++;

        // XXX at this point, we should check if the
        // thing after the ctrl-D is an ordinary line-mode command
        // or not. If it is, then we should handle it. But for now,
        // we will punt: the only way for multiple commands to end
        // up in one packet is if there is a very high transmit rate.
        // And this is improbable for teh current server design/usage.
    }

    // If we are here, then the last char in the buffer was a ctrl-D. 
    // (That's the only way out of the while loop above). Go back to
    // line-mode.
    SetLineProtocol(true);
    buffer = "";
    in_raw_mode = false;
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
            sock->Send(line + "\r\n");
        }
        sock->Send(master->getCommandPrompt());
    }
    pthread_mutex_unlock(&sock_lock);

    // There will be one callback for each request that 
    // was generated on this socket. When all of these
    // have been processed, then self-delete, as no one
    // else will.
    int cnt = AtomicInc(-1);
    if (cnt <= 0) delete this;
}
