/*
 * opencog/embodiment/AGISimSim/proxy/SimClientSocket.cc
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
#include "SimClientSocket.h"
#include "util/Logger.h"

SimClientSocket::SimClientSocket(ISocketHandler& h, AsynchronousMessageReceiver* _receiver, bool echoing)
:TcpSocket(h)
{
	m_bEchoing = echoing;
    m_bWaitingForResponse = false;
    m_bConnectionFailed = false;
    receiver = _receiver;
	SetLineProtocol();

    // TODO: create a thread to periodically call h->Select() method and read spontaneous messages from AGISIM.
}


SimClientSocket::~SimClientSocket()
{
}

void SimClientSocket::OnConnectFailed()
{
	printf("SimClientSocket::OnConnectFailed()!!!\n");
	m_bWaitingForResponse = false;
    m_bConnectionFailed = true;
    m_sResponse = "";
}

bool SimClientSocket::ConnectionFailed() {
	return m_bConnectionFailed;
}

void SimClientSocket::OnConnect()
{
	printf("SimClientSocket::OnConnect()!!!\n");
}

bool SimClientSocket::isWaitingForResponse() {
	return m_bWaitingForResponse;
}

void SimClientSocket::cancelWaitingForResponse() {
    m_bWaitingForResponse = false;
}

void SimClientSocket::markWaitingForResponse() {
    m_bWaitingForResponse = true;
}


std::string SimClientSocket::getResponse() {
    return m_sResponse;
}

void SimClientSocket::Send(const std::string& str)
{
    TcpSocket::Send(str);
}


void SimClientSocket::OnLine(const std::string& line)
{
    opencog::logger().log(opencog::Logger::DEBUG, "SimClientSocket::OnLine: init");

    if (opencog::logger().getLevel() >= opencog::Logger::FINE)
        opencog::logger().log(opencog::Logger::FINE, "SimClientSocket::OnLine: %s", line.c_str());
  // TODO: More than one response can arrive for a same command!
  //       How to handle this? 
  //       a) Get only the first response 
  //       b) Get only the last (in this case, there could be synchonization problems, 
  //          since SimProxy's send() code does not know if there will be more responses) 
  //       c) Handle the responses independently (calling a method of a Listener registered in the SimClientSocket).
  //
  // CURRENT IMPLEMENTATION IS A MIX OF (a) and (c). CALLER CAN/MUST ALSO CANCEL RESPONSE RECEPTION IN SPECIFIC SITUATIONS (TIMEOUT, FOR EXAMPLE). 
  //
  timeval theTime;
  gettimeofday(&theTime, NULL);
  if (m_bEchoing)
	  printf("AGISim Server (@%ld.%ld ms) >> '%s'\n", theTime.tv_usec/1000, theTime.tv_usec%1000, line.c_str());
  if (strstr(line.c_str(), "</sensation>") != NULL || 
      strstr(line.c_str(), "</agisim>") != NULL ) 
  {
    if (m_bWaitingForResponse) {
        m_bWaitingForResponse = false;
        m_sResponse = line;
    } else {
        if (receiver != NULL) {
            receiver->receiveAsynchronousMessage(line);
        }
    }
  }
  else
  {
//    printf("Invalid message received: %s\n", line.c_str());
//	SetCloseAndDelete();
  }
}


