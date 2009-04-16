/*
 * opencog/embodiment/AGISimSim/server/src/serversocket.cpp
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

/**

Copyright Ari A. Heljakka 03/2006
Not part of AGI-Sim.
Do not distribute.

*/

/***************************************************************************
 *  ServerSocket & command implementations.        
 *
 *  Project: Novamente
 ****************************************************************************/


#include "simcommon.h"
#include "space.h"
#include "XMLNode.h"
#include "serversocket.h"
#include "command.h"
#include "simserver.h"

#if CRYSTAL
 #include "CSserver.h"
 #include "CSMeshTool.h"
#endif

#include "CSworld.h"
#include "simworld.h"
#include "sensors.h"
#include "CSagent.h"

/* Socket Thread */

#include <boost/thread/thread.hpp>
#include <stack>

#ifdef WIN32
	#include <sys/timeb.h>
#endif

	typedef unsigned int  uint;

//----------------------------------------------------------------------------------------------------------------

bool  SelectCommand( string  vcmd, ReportProvider*  reporter, shared_ptr< Command >&  com, ServerSocket*  _socket = NULL);

//--------------------------------------------------------------------------------------------------------------
iMeshWrapper*  CSWorld::GetObject (std::string meshname) {
    iEngine*  engine = FROM_CS( iEngine );
    iMeshWrapper*   mesh   = engine->FindMeshObject( meshname.c_str() );

    return mesh;
}

//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
bool  SocketManager::SendMsg( Socket*  socket, string  msg ) {
	boost::mutex::scoped_lock  lock( lock_provider );
	
	if ( STLhas(sockets, socket) )  {
		ServerSocket*  s = dynamic_cast<ServerSocket*>( socket );
		
		if ( s ) {
			s->SendMsg(msg);
			return true;
		}
		else  { 
			LOG( "ServerSocketHManager", 0, "Only ServerSockets should be used!" ); 
		}			
	}
	else  {
		LOG( "SocketManager", 2, "Tried to send data to non-existent socket." ); 
	}
	
	return false;
}

//----------------------------------------------------------------------------------------------------------------
bool SocketManager::SendError( Socket*  socket, string  msg )
{
	boost::mutex::scoped_lock  lock(lock_provider);
	
	if ( STLhas(sockets, socket) )	{
		ServerSocket*  s = dynamic_cast<ServerSocket*>( socket );
		
		if ( s ) {
			s->SendError(msg);
		}
		else  {
			LOG( "ServerSocketHManager", 0, "Only ServerSockets should be used!" ); 
		}			
	}
	else  {  
		LOG( "SocketManager", 2, "Tried to send data to non-existent socket." ); 
	}
	
	return true;
}

//----------------------------------------------------------------------------------------------------------------
long getusec();

//----------------------------------------------------------------------------------------------------------------
bool  ServerSocket::SendMsg( string  msg )
{
	EnqueueSend( XMLembed( "agisim", XMLembed( "msg", msg ) ) );
	return true;
}

//----------------------------------------------------------------------------------------------------------------
bool  ServerSocket::SendError( string  msg )
{
	EnqueueSend( XMLembed( "agisim", XMLembed( "error", msg ) ) );
	 LOG( "ServerSocket", 1, "EnQueued the error msg: " + msg);
	return true;
}

//----------------------------------------------------------------------------------------------------------------
ServerSocket::ServerSocket( ISocketHandler&  h )
						  :GenericSocket( h )
{
	SetLineProtocol();
}

//----------------------------------------------------------------------------------------------------------------
void ServerSocket::Init()
{
	GenericSocket::Init();
}

//----------------------------------------------------------------------------------------------------------------
void  ServerSocket::OnRead()
{
	TcpSocket::OnRead();
	 LOG( "ServerSocket", 10, "Read event." );
}

//----------------------------------------------------------------------------------------------------------------
void  ServerSocket::OnWrite()
{
	TcpSocket::OnWrite();
	 LOG( "ServerSocket", 10, "Write event." );
}

//----------------------------------------------------------------------------------------------------------------
ServerSocket::~ServerSocket()
{
}

//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
//The cmd must be "command [args]"
extern timeval  last_com;
extern timeval  last_response;
extern boost::mutex  perflock_provider;

//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
class LogReportProvider2 : public ReportProvider
{
public:
	LogReportProvider2() {
	}

	virtual void  SendMsg(string msg)
	{
		 LOG( "LogReportProvider", 1, msg);
	}

	virtual void  SendError(string msg)
	{
		 LOG( "LogReportProvider", 0, msg);
	}
};

//----------------------------------------------------------------------------------------------------------------
bool  SelectCommand(string  cmd, ReportProvider*  reporter, shared_ptr< Command >&  com,  ServerSocket*  _socket)
{
    // Tokenize the command
    StringTokenizer  tok (cmd, "\t\n\r (,);" );
    // Clean the command
    vector< std::string >  vcmd = tok.WithoutEmpty();

    if ( vcmd.empty() )  {
        reporter->SendError( "Empty command." );
        return false;
    }
    
    // Reconstruct the rest of the command line into an argument string
    std::string  args;
    for (uint  i = 1; i < vcmd.size(); i++)
        args += vcmd[i] + " ";

     LOG( "ServerSocket",3, "Parsed into: " + vcmd[0] + "// " + args + " //" );
    
    if ( vcmd[0] == "new" ) com = shared_ptr<Command>( new CommandNew( reporter, _socket ) );
    else if ( vcmd[0]  == "delete" //Some fun with English, yes?
            || vcmd[0] == "del"
            || vcmd[0] == "rm"
            || vcmd[0] == "remove"
            || vcmd[0] == "erase"
            || vcmd[0] == "kill" ) com = shared_ptr< Command >(new CommandDelete(reporter));
    else if ( vcmd[0] == "update") com = shared_ptr< Command >( new CommandUpdate (reporter) );
    else if ( vcmd[0] == "dump"  ) com = shared_ptr< Command >( new CommandDump     (reporter) );
    else if ( vcmd[0] == "sense"   ) com = shared_ptr< Command >( new CommandSense  (reporter) );
    else if ( vcmd[0] == "explain" ) com = shared_ptr< Command >( new CommandExplain(reporter) );
    else if ( vcmd[0] == "reset"   ) com = shared_ptr< Command >( new CommandReset  (reporter) );    
    else if ( vcmd[0] == "action"  ) com = shared_ptr< Command >( new CommandAction (reporter) );
    else if ( vcmd[0] == "config"  ) com = shared_ptr< Command >( new CommandConfig (reporter) );
    else if ( vcmd[0] == "listen"  ) com = shared_ptr< Command >( new CommandListen (reporter, _socket) );
    else if ( vcmd[0] == "mark"    ) com = shared_ptr< Command >( new CommandMark   (reporter) );
    else if ( vcmd[0] == "paint"   ) com = shared_ptr< Command >( new CommandPaint  (reporter) );
    else if ( vcmd[0] == "test"    ) com = shared_ptr< Command >( new CommandTest   (reporter, _socket) );
    else if ( vcmd[0] == "meshes"  ) com = shared_ptr< Command >( new CommandMeshes (reporter) );
    else if ( vcmd[0] == "getpos"  ) com = shared_ptr< Command >( new CommandGetPos (reporter) );
    else if ( vcmd[0] == "setpos"  ) com = shared_ptr< Command >( new CommandSetPos (reporter));
    else if ( vcmd[0] == "getrot"  ) com = shared_ptr< Command >( new CommandGetRot (reporter));
    else if ( vcmd[0] == "setrot"  ) com = shared_ptr< Command >( new CommandSetRot (reporter));
    else if ( vcmd[0] == "getsize"  ) com = shared_ptr< Command >( new CommandGetSize (reporter) );
    else if ( vcmd[0] == "say"
            || vcmd[0] == "broadcast"
            || vcmd[0] == "msg"
            || vcmd[0] == "message") com = shared_ptr< Command >( new CommandBroadcast( reporter, _socket ) );
    else if ( vcmd[0] == "rotate"  ) com = shared_ptr< Command >( new CommandRotate   ( reporter, _socket ) );
    else if ( vcmd[0] == "relight" ) com = shared_ptr<Command>(new CommandRelight(reporter, _socket));
    else if ( vcmd[0] == "light"   ) com = shared_ptr<Command>(new CommandLight(reporter, _socket));
    else if ( vcmd[0] == "lift"   ) com = shared_ptr<Command>(new CommandLift(reporter, _socket));
    else if ( vcmd[0] == "drop"   ) com = shared_ptr<Command>(new CommandDrop(reporter, _socket));
    else if ( vcmd[0] == "g"   ) com = shared_ptr<Command>(new CommandGraphDump(reporter, _socket));
    else if ( vcmd[0] == "move.arm" ) com = shared_ptr<Command>(new CommandMoveArm(reporter, _socket));
    else if ( vcmd[0] == "move.leg" ) com = shared_ptr<Command>(new CommandMoveLeg(reporter, _socket));
    //else if ( vcmd[0] == "close"  ) com= shared_ptr< Command >(new CommandQuit(reporter)); 
    else
        return false;
    
    com->addArguments(args);
    
    return true;
}

//----------------------------------------------------------------------------------------------------------------
void ServerSocket::OnLine( const std::string&  cmd )
{
    //printf( "Received msg\n%s\n", cmd.c_str());
	timeval  this_com;
#ifdef WIN32
	struct _timeb  timebuffer;
    _ftime( &timebuffer );
	this_com.tv_sec  = timebuffer.time;
	this_com.tv_usec = timebuffer.millitm;
#else
	gettimeofday( &this_com, NULL );
#endif
	
	 LOG( "ServerSocket",1, "Received: " + cmd);			
	shared_ptr< Command >  com;	
	SocketReportProvider*  reporter = new SocketReportProvider(this);
	
	if ( !SelectCommand( cmd, reporter, com, this ) )  {
		 reporter->SendError( "Unknown command" );
		return;
	}

	boost::mutex::scoped_lock  lock( perflock_provider );
	
	// = if no calls are pending
	if ( last_com.tv_sec  == last_response.tv_sec  && 
		 last_com.tv_usec == last_response.tv_usec)
	{
		last_com = this_com;		
		// LOG( "GenericSocket", 1, "LC "+i2str(last_com.tv_usec));		
	}

	lock.unlock();
	
	SimServer::Get()->EnqueueCommand( com ); //Parse later
}

//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
OperationSocket::OperationSocket(ISocketHandler& h)
: ServerSocket(h)
{ }

OperationSocket::~OperationSocket()
{ }


//----------------------------------------------------------------------------------------------------------------
void  OperationSocket::Init()
{
	ServerSocket::Init();
	 LOG( "ServerSocket", 3, "Init event." );

	/*	commands["new"] = new CommandNew(this);
		commands["delete"] = new CommandDelete(this);
		commands["reset"] = new CommandReset(this);
		commands["close"] = new CommandQuit(this);
		commands["action"] = new CommandAction(this);*/
	//		shared_ptr< Agent >((Agent*)(NULL))); //(new CSAgent(NULL, NULL, ));
		
	// #warning "NULL agent created while the socket has no real agent!"
		
	//		shared_ptr< Agent >((Agent*)(SimServer::Get()->GetAgent())));
}


//----------------------------------------------------------------------------------------------------------------
/*
DemonSocket::DemonSocket(SocketHandler&)
: ServerSocket(h)
{ }

DemonSocket::~DemonSocket()
{ }

void DemonSocket::Init()
{
//	TcpSocket::Init();
	ListenSocket< TcpSocket >::Init();

	 LOG( "ServerSocket", 3, "Init event." );

	commands["action"] = new CommandAction(this, 
		shared_ptr< Demon >(SimServer::Get()->GetDemon());
}*/
