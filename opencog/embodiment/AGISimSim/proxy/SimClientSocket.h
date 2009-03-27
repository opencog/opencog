/*
 * opencog/embodiment/AGISimSim/proxy/SimClientSockect.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by TO_COMPLETE
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
#ifndef _SIMCLIENTSOCKET_H
#define _SIMCLIENTSOCKET_H

#include <Sockets/TcpSocket.h>

#include "AsynchronousMessageReceiver.h"

#include <string>

class ISocketHandler;
class TcpSocket;

class SimClientSocket : public TcpSocket
{
public:
	SimClientSocket(ISocketHandler&, AsynchronousMessageReceiver* receiver, bool echoing = false);
	~SimClientSocket();

	// Overiden methods
	void OnConnect();
	void OnConnectFailed();
	void OnLine(const std::string&);
	void Send(const std::string &str);
    void markWaitingForResponse();

    // Specific methods
	bool isWaitingForResponse();
    void cancelWaitingForResponse();
	std::string getResponse();
    bool ConnectionFailed();

private:
    bool m_bEchoing;
	bool m_bWaitingForResponse;
	bool m_bConnectionFailed;
	std::string m_sResponse;
    AsynchronousMessageReceiver* receiver;
};




#endif // _SIMCLIENTSOCKET_H
