/***************************************************************************
 *  Socket classes.        
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *																			
 *	19.01.06	FP	formatting 												
 ****************************************************************************/

#ifndef _SERVERSOCKET_H
#define _SERVERSOCKET_H

#include "simcommon.h"
#include <queue>
#include <boost/thread/condition.hpp>

#include <Sockets/TcpSocket.h>
#include <Sockets/SocketHandler.h>
#include <Sockets/ListenSocket.h>

//------------------------------------------------------------------------------------------------------------
/** @class StatusHandler
	\brief A handler class for multiple simultaneous sockets. */
//------------------------------------------------------------------------------------------------------------
class StatusHandler : public SocketHandler {
public:
	StatusHandler();

	void tprintf(TcpSocket *,char *format, ...);
	void List(TcpSocket *);
	void Update();
	void UndertakeTheVitallyChallenged();
	void Disconnect();
};

//------------------------------------------------------------------------------------------------------------
/** @class ServerSocketHandler
	\brief Status handler for the server sockets.*/
//------------------------------------------------------------------------------------------------------------
class ServerSocketHandler : public StatusHandler {
public:
};

//------------------------------------------------------------------------------------------------------------
/** @class GenericSocket
	\brief The generic TCP socket, capable of queuing traffic. */
//------------------------------------------------------------------------------------------------------------
class GenericSocket : public TcpSocket { //: public ListenSocket<TcpSocket>
protected:
	/** Can only be used from SocketManager */
	bool 					 Flush();
	std::queue<std::string>  outgoing;
	boost::mutex 			 lock_provider;
public:
	GenericSocket(ISocketHandler& h);
	virtual ~GenericSocket();

	virtual void Init();
	virtual void OnConnect();
	virtual void OnAccept();
	/** Enqueue the message for sending it later */
	void EnqueueSend(std::string msg);
	/** Delete all queued messages */
	void ClearQueue();

	friend class SocketOutputThread;
	friend class SocketManager;
};

class Command;

//------------------------------------------------------------------------------------------------------------
/** @class ServerSocket
	\brief The server-side TCP socket.*/
//------------------------------------------------------------------------------------------------------------
class ServerSocket : public GenericSocket { //: public ListenSocket<TcpSocket>
protected:
	//	typedef std::map<std::string, shared_ptr<SocketCommand> > commap;
	typedef std::map<std::string, Command* > commap;

	commap commands;
	bool   SendMsg  (string msg);
	bool   SendError(string msg);
public:
	ServerSocket(ISocketHandler& );
	~ServerSocket();

	virtual void Init   ();
	virtual void OnRead ();
	virtual void OnWrite();
			void OnLine	(const std::string& );

	friend class SocketManager;	
	//	const ServerSocketHandler& Handler2() const;
};

//------------------------------------------------------------------------------------------------------------
/** @class OperationSocket
	\brief The server-side TCP socket for all operations.
	TODO: should have separate sockets for demon and administration operations.*/
//------------------------------------------------------------------------------------------------------------
class OperationSocket : public ServerSocket {	
public:
	OperationSocket(ISocketHandler&);
	~OperationSocket();
	virtual void Init();
};

/*
class DemonSocket : public ServerSocket
{
	
public:
	DemonSocket(SocketHandler&, shared_ptr<Agent> _agent);
	~DemonSocket();
	virtual void Init();
};
*/

//------------------------------------------------------------------------------------------------------------
/** @class SocketManager
	\brief Manages all sockets.*/
//------------------------------------------------------------------------------------------------------------
class SocketManager : public Singleton<SocketManager> {
protected:
	bool running;
public:
	set<int> 	  ports;
	set<Socket*>  sockets;
	boost::mutex  lock_provider;
	boost::condition  condition_provider;
	
	SocketManager() : running(false) { }
	~SocketManager();
	
	/** Each socket must be registered before it can be used.
		Each socket should automatically register itself.	*/
	
	void Register	  (Socket* socket, bool exist = true);	
	bool IsRegistered (Socket* socket);	
	bool SendMsg	  (Socket* socket, string msg);
	bool SendError    (Socket* socket, string msg);

	/** Delete all queued messages in all sockets */
	void ClearAll();	
	/** Send all queued messages in all sockets */
	void FlushAll();
	
	friend class Singleton<SocketManager>;
	friend class SocketOutputThread;
};

#define TheSocketManager (SocketManager::Instance())

#endif // _SERVERSOCKET_H
