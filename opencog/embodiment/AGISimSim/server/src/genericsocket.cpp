/*
 * opencog/embodiment/AGISimSim/server/src/genericsocket.cpp
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari A. Heljakka
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

/*	This software uses Boost.threads.
	Boost.threads is Copyright © 2001-2003 William E. Kempf.
	Permission to use, copy, modify, distribute and sell this software and its documentation for any purpose is hereby granted without fee, provided that the above copyright notice appear in all copies and that both that copyright notice and this permission notice appear in supporting documentation. William E. Kempf makes no representations about the suitability of this software for any purpose. It is provided "as is" without express or implied warranty.
*/


#include "simcommon.h"
#include "serversocket.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

//For performance measurements:
//#include <sys/time.h>
#include <time.h>

#ifdef WIN32
#include <sys\timeb.h>
#endif

using std::string;
using std::set;

timeval last_com;
timeval last_response;
long    total_process_time = 0;

boost::mutex outlock_provider;

//------------------------------------------------------------------------------------------------------------
void SocketManager::ClearAll () {
	boost::mutex::scoped_lock lock(lock_provider);
	
	for (set<Socket*>::iterator  it = sockets.begin(); it != sockets.end();	it++) {
		GenericSocket* s = dynamic_cast<GenericSocket*>((*it));
		
		if (s) {
			 LOG("ServerSocketManager", 5, "Clearing..");
			s->ClearQueue();
			 LOG("ServerSocketManager", 5, "Cleared");
		}
		else LOG("ServerSocketManager", 0, "Only ServerSockets should be used!"); 
	}
}

//------------------------------------------------------------------------------------------------------------
SocketManager::~SocketManager () {
	ClearAll();
}

//------------------------------------------------------------------------------------------------------------
void SocketManager::FlushAll ()
{
	 LOG("SocketOutputThread", 4, "About to flush sockets, but waiting for socket lock to open.");
	
	// Locks the sockets
	boost::mutex::scoped_lock sm_lock(lock_provider);

	 LOG("SocketOutputThread", 4, "Flushing " +toString(sockets.size()) + " sockets.");

	for (set<Socket*>::iterator  it = sockets.begin(); it != sockets.end();	it++) {
		GenericSocket* s = (GenericSocket*)(*it);
		
		if (s)	s->Flush();
		else    LOG("SocketOutputThread", 0, "Only ServerSockets should be used!"); 
	}

	 LOG("SocketOutputThread", 4, "Flushed all.");
}

//------------------------------------------------------------------------------------------------------------
bool SocketManager::IsRegistered (Socket* socket) {
	boost::mutex::scoped_lock lock(lock_provider);
	
	return STLhas(sockets, socket);
}

//------------------------------------------------------------------------------------------------------------
void SocketManager::Register (Socket* socket, bool exist) {
	boost::mutex::scoped_lock lock(lock_provider);

	if (exist) {
		sockets.insert(socket);
		// Launch the output thread
		
		if (!running) {
			running = true;			
			 LOG("SocketManager::Register", 4, "Try unlock.");
			
			// No more internal operations will happen here,
			// so we can release the lock.
			lock.unlock(); 

			/*LOG("SocketManager::Register", 4, "Begin output thread.");
			// The thread will re-lock.
			SocketOutputThread flusher(this);
			boost::thread t(flusher);
			 LOG("SocketManager::Register", 4, "Output thread running.");*/
		}
	}
	else sockets.erase(socket);
}

//------------------------------------------------------------------------------------------------------------
void StatusHandler::UndertakeTheVitallyChallenged () {
	for (socket_m::iterator it = m_sockets.begin(); it != m_sockets.end(); it++) {
		Socket *p = (*it).second;
		if (p->CloseAndDelete())  p->Close();
	}
}

//------------------------------------------------------------------------------------------------------------
StatusHandler::StatusHandler() : SocketHandler () {
	last_com.tv_sec = last_com.tv_usec = last_response.tv_sec = last_response.tv_usec = 0;
}

//------------------------------------------------------------------------------------------------------------
/*void StatusHandler::tprintf(TcpSocket *p,char *format, ...)
{
	va_list ap;
	size_t n;
	char tmp[SIZE];

	va_start(ap,format);
	n = vsnprintf(tmp,SIZE - 1,format,ap);
	va_end(ap);

	p -> SendBuf(tmp, strlen(tmp));
}*/

//------------------------------------------------------------------------------------------------------------
void StatusHandler::List (TcpSocket *sendto) {
	char buf[64000];
	
	sprintf(buf, "Handler Socket List\n");
	for (socket_m::iterator it = m_sockets.begin(); it != m_sockets.end(); it++) {
		Socket    *p  = (*it).second;
		TcpSocket *p3 = dynamic_cast<TcpSocket *>(p);

		sprintf (buf, " %s:%d", p -> GetRemoteAddress().c_str(), p -> GetRemotePort());
		sprintf (buf, "  %s  ", p -> Ready() ? "Ready" : "NOT_Ready");
		sprintf (buf, "\n");
		sprintf (buf, "  Uptime:  %d days %02d:%02d:%02d\n",
				(int)( p -> Uptime() / 86400),
				(int)((p -> Uptime() / 3600) % 24),
				(int)((p -> Uptime() / 60) % 60),
				(int)( p -> Uptime() % 60));
		if (p3)	{
			sprintf(buf, "    Bytes Read: %9lu\n", p3 -> GetBytesReceived());
			sprintf(buf, "    Bytes Sent: %9lu\n", p3 -> GetBytesSent());
		}
	}
	sprintf(buf, "\n");
	
	sendto->SendBuf(buf, strlen(buf));
}

//------------------------------------------------------------------------------------------------------------
void StatusHandler::Update () {
	for (socket_m::iterator it = m_sockets.begin(); it != m_sockets.end(); it++) {
		Socket    *p0 = (*it).second;
		TcpSocket *p  = dynamic_cast<TcpSocket*>(p0);
		if (p) 	List(p);
		
	}
}

//------------------------------------------------------------------------------------------------------------
void StatusHandler::Disconnect () {
	for (socket_m::iterator it = m_sockets.begin(); it != m_sockets.end(); it++) {
		Socket    *p0 = (*it).second;
		TcpSocket *p  = dynamic_cast<TcpSocket *> (p0);
		
		if (p && p->Uptime() > 60 )	{
			char buf[] = "Goodbye\n";
			p->SendBuf(buf, strlen(buf));
			p->SetCloseAndDelete();
		}
	}
}

//------------------------------------------------------------------------------------------------------------
void GenericSocket::EnqueueSend (std::string msg)
{
	 LOG("GenericSocket", 4, "Enquing msg. Passing lock...");
	
	boost::mutex::scoped_lock lock(lock_provider);
	
	 LOG("GenericSocket", 4, "Lock passed.");

	if (CloseAndDelete())	{
		 LOG("GenericSocket",4, "Socket is dead. Aborting.");
		return;
	}
	
	outgoing.push(msg);
}

boost::mutex perflock_provider;

//------------------------------------------------------------------------------------------------------------
bool GenericSocket::Flush () {
	if (CloseAndDelete()) {
		 LOG("GenericSocket",4, "Socket is dead. Aborting.");
		return false;
	}
	
	try	{
		string msg;
		
		while (!outgoing.empty()) {
			msg += outgoing.front() + "\n";
			outgoing.pop();
		}

		 LOG("GenericSocket", 3, "Sending msg... "); //+msg );		
		 //    printf("******** Sending Msg ... size = %d\n", msg.size());
		Send(msg);
		
//#ifndef WIN32
		// Performance measures:		
		boost::mutex::scoped_lock lock (perflock_provider);
#ifdef WIN32
   struct _timeb timebuffer;
   _ftime( &timebuffer );
	last_response.tv_sec = timebuffer.time;
	last_response.tv_usec = 1000*timebuffer.millitm;
#else

		gettimeofday (&last_response, NULL);
	
#endif
		 // LOG("GenericSocket", 3, "LR "+toString(last_response.tv_usec));
		
		int sec  = last_response.tv_sec - last_com.tv_sec;
		int usec = last_response.tv_usec - last_com.tv_usec;		
		last_com = last_response;
		
		lock.unlock();
		
		long dusec = sec*1000*1000+usec;
		if (dusec > 0) {
			total_process_time += dusec;
			 LOG("GenericSocket", 3, "Send ok. Total process time: "
				  + toString (total_process_time/1000000) + "s "
				  + toString (total_process_time%1000000) + "us");
		}
		else {
			LOG("GenericSocket", 1, "NEGATIVE PROCESSING TIME???");
		}
//#endif


		return true;
	} catch(...) {
		 LOG("ServerSocket", 0, "Exception in sending. Sends may've been wasted.");
		outgoing.pop(); //Remove the problem child
		return false;
	}
}

//------------------------------------------------------------------------------------------------------------
GenericSocket::GenericSocket (ISocketHandler& h)
							  :TcpSocket(h)
{
}

//------------------------------------------------------------------------------------------------------------
GenericSocket::~GenericSocket() {
	string  myname = "ServerSocket #" + toString(GetParent()?GetParent()->GetPort():0);
	
	SetCloseAndDelete();

	boost::mutex::scoped_lock lock (lock_provider);
	
	TheSocketManager.Register (this, false);
	 LOG(myname, 3, "Un-Registered.");
}

//------------------------------------------------------------------------------------------------------------
void GenericSocket::OnConnect() {
	TcpSocket::OnConnect();
	string  myname = "ServerSocket #" + toString(GetParent()?GetParent()->GetPort():0);

	if (!TheSocketManager.IsRegistered (this)) {
		TheSocketManager.Register (this);
		 LOG(myname, 1, "NEW: Registered with CONNECT event.");
	}
	else  LOG(myname, 1, "NEW: Tried to re-register this socket!");
}

//------------------------------------------------------------------------------------------------------------
void GenericSocket::OnAccept() {
	TcpSocket::OnAccept();
	string  myname = "ServerSocket #" + toString(GetParent()?GetParent()->GetPort():0);
	
	 LOG(myname, 2, "Accept event.");
}

//------------------------------------------------------------------------------------------------------------
void GenericSocket::Init() {
	TcpSocket::Init();
	string  myname = "ServerSocket #" + toString(GetParent()?GetParent()->GetPort():0);
	TheSocketManager.Register(this);

	 LOG(myname,	1, "NEW: Registered with INIT event.");
}

//------------------------------------------------------------------------------------------------------------
void GenericSocket::ClearQueue() {
	string  myname = "ServerSocket #" + toString(GetParent()?GetParent()->GetPort():0);

	 LOG(myname,4, "ClearQueue locking...");
	
	boost::mutex::scoped_lock lock(lock_provider);

	 LOG(myname,4, "ClearQueue lock passed.");
	
	if (CloseAndDelete()) {
		 LOG(myname,4, "Socket is dead. Aborting.");
		return;
	}
	
	while (!outgoing.empty())  outgoing.pop();
}
