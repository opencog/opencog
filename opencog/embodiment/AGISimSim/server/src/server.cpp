
/**

Copyright Ari A. Heljakka 03/2006
Not part of AGI-Sim.
Do not distribute.

*/


/***************************************************************************
 *  Server operations.
 * 
 *  Project: Novamente
 *  Fri Feb 18 11:35:16 2005
 ****************************************************************************/


/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA. */

/*  This software uses Boost.threads.
    Boost.threads is Copyright © 2001-2003 William E. Kempf.
    Permission to use, copy, modify, distribute and sell this software and its documentation for any purpose is 
    hereby granted without fee, provided that the above copyright notice appear in all copies and that both that 
    copyright notice and this permission notice appear in supporting documentation.  
    William E. Kempf makes no representations about the suitability of this software for any purpose. 
    It is provided "as is" without express or implied warranty.
*/

#include "simcommon.h"
#include "space.h"
#include "command.h"
#include "simworld.h"
#include "demon.h"
#include "CSagent.h"
#include "CSworld.h"
#include "simserver.h"
#include "collisionsystem.h"
#include "sensors.h"

/* Socket Thread */
#include <boost/thread/thread.hpp>
#include "serversocket.h"

#ifdef WIN32
    #include <sys/timeb.h>
#endif

long  lastflush = -1;
long  wasted    =  0;

//-----------------------------------------------------------------------------------------------------------------
long getusec()
{
    timeval  theTime;

#ifdef WIN32
    struct _timeb  timebuffer;
    _ftime( &timebuffer );
    // theTime.tv_sec = timebuffer.time;
    theTime.tv_usec = timebuffer.millitm;
#else
    gettimeofday( &theTime, NULL );
#endif

    return theTime.tv_usec;
}

//-----------------------------------------------------------------------------------------------------------------
struct ServerSocketThread
{
    void operator() ()    {
         LOG( "SocketManager", 1, "initialized." );
        ServerSocketHandler  h;

        // TODO: Leaks sockets.        
        
        // OperationSocket l(h);        
         LOG( "SocketManager", 1, "Binding sockets: " + toString( IntConfig( "sockets" ) ));
        for (int s = 0; s < IntConfig( "sockets" );s++) {
            ListenSocket< OperationSocket >*  l = new ListenSocket< OperationSocket >( h );            
             LOG( "SocketManager", 1, "Binding socket #" + toString(40001+s));
            
            if ( l->Bind(40001 + s,10) ) {
                exit(-1);
            }
            h.Add( l );
        }
        
        h.Select( 1, 0 );
        
        while ( h.GetCount() )  {
             LOG( "SocketManager", 4, "Polling..." );
            h.Select(0, IntConfig( "SocketPollingDelay" ) * 1000 ); // (sec, microsec)
             LOG( "SocketManager", 4, "Not Polling." );
        }

         LOG( "SocketManager", 1, "Count Zero." );    
    }
};

//-----------------------------------------------------------------------------------------------------------------
void NewSocketThread()
{
    ServerSocketThread  st;
    boost::thread  t( st );
     LOG( "RunThread", 0, "out..." );
}

//-----------------------------------------------------------------------------------------------------------------
/** RunSocketThread begins the thread which calls ServerSocket::Online
    whenever a socket receives a command.

    Online() transforms the command into a SocketCommand object and
    stores it into the thread-safe Server::socketCommands queue.

    Upon a call from CS::SetupFrame, Server::OnIdleEvent() launches
    parse() method of each pending SocketCommand.

    SocketCommand.parse() then carries out the actual command.
    
    The reason we won't carry out the commands right away is simply
    thread-safety and overall greater control over the environment.
    The server decides when things take place.
*/
//-----------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------
void hacklog( string  msg, ServerSocket*  socket)
{
    if ( socket == NULL ) {
        if ( !TheSocketManager.sockets.empty() )  {
             LOG( "hack LOG", 3, "sending msg to socket" );
            ServerSocket* s = dynamic_cast<ServerSocket*>(*TheSocketManager.sockets.begin());
             // printf( "******** Msg enqueued... size = %d\n", msg.size());
            s->EnqueueSend(msg);
             LOG( "hack LOG", 4, "Sent: " + msg );
        }
        else {
             LOG( "hack LOG", 1, "Tried to send data but no sockets exist!" );
        }
    } 
    else if ( TheSocketManager.IsRegistered( socket ) ) {
         LOG( "hack LOG", 3, "sending msg to specific socket" );
        socket->EnqueueSend(msg);        
         LOG( "hack LOG", 4, "Sent: " + msg);
    } 
    else {
         LOG( "hack LOG", 3, "specific socket does not exist anymore" );
    }
}

//-----------------------------------------------------------------------------------------------------------------
/** These will be called serially from Server::OnIdleEvent(),
    so all we do here is thread-safe.*/
//-----------------------------------------------------------------------------------------------------------------
PosListener::PosListener( LocalObj3D&  _obj, LocalServer&  _server)
                        : obj( _obj ), server( _server ), currently_checking_collisions( false )
{
}

//-----------------------------------------------------------------------------------------------------------------
void LocalServer::EnqueueCommand( shared_ptr< Command >  sc )
{
    boost::mutex::scoped_lock  lock( socket_lock_provider );    

    socketCommands.push(sc);    

    lock.unlock();

     LOG( "Server", 3, "Send frame update hint..." );

    // TODO: Performance issue here?
    
    // New commands will be processed only in the idle event driven by the frame system, so we hint the frame pump to update it.
    cs->GUIprovider->PleaseUpdateFrame();

    //Before the hint's been noticed, we have time to deque. The frame update will not continue before the deque mutex is free.
    DequeueCommands();
}

//-----------------------------------------------------------------------------------------------------------------
void LocalServer::EnqueueHighPriorityCommand( shared_ptr< Command >  sc)
{
    //printf("LocalServer::EnqueueHighPriorityCommand() start\n");
    highPrioritySocketCommands.push(sc);    
    //printf("LocalServer::EnqueueHighPriorityCommand() end\n");
}


//-----------------------------------------------------------------------------------------------------------------
boost::mutex&  LocalServer::AcquireLock()
{
    return socket_lock_provider;
}

//-----------------------------------------------------------------------------------------------------------------
// Should only be used from the script loader (script world) 
//-----------------------------------------------------------------------------------------------------------------
void  LocalServer::PerformCommand( shared_ptr< Command >  sc)
{
    boost::mutex::scoped_lock  lock( socket_lock_provider );
    
    if ( IntConfig( "FrameUpdateDelay" ) )  {
        // LOG( "Server",0,"INTERNAL ERROR! if FrameUpdateDelay!=0, you should not perform commands directly!" );
        // return;
    }
     LOG( "Server", 3, "Parse a socket command..." );

    try    {
        sc->parse();
         LOG( "Server", 3, "Parse ok." );
        
    } catch(...) {
         LOG( "Server", 2, "Exception in parsing command." );
    }
}

//-----------------------------------------------------------------------------------------------------------------
void LocalServer::DequeueCommands()
{
    boost::mutex::scoped_lock  lock( socket_lock_provider );

     LOG( "Server", 3, "Dequeue socket commands..." );
    
    try {
        if ( socketCommands.size() < 2 )  { //Measure the time elapsed since last flush.
            //We're still waiting for new commands, so the meantime is wasted.
            if (lastflush != -1)  {
                long  now        = getusec();
                long  now_wasted = now - lastflush;
            
                if ( now_wasted < 0 ) //Wrapped the second line
                    now_wasted += 1000000;
                
                lastflush = now;
                wasted   += now_wasted;            
                 LOG( "Server", 4, "Socket waste time now: " + toString(now_wasted) );
            }
        }        
        
        while ( !highPrioritySocketCommands.empty() )  {
            shared_ptr< Command >  com = highPrioritySocketCommands.front();
            
            long  before = getusec();
            
             LOG( "Server", 3, "Parsing commands from high priority queue... ( " +toString(highPrioritySocketCommands.size()) + " left)" );

            if (com->parse() == PARSE_NOT_PROCESSED) {
                printf("Command not processed for some reason. Kept in the commands queue for further processing\n");
		break;
            }  else {
                // Only removes from queue if it has been really processed.
                highPrioritySocketCommands.pop();
            }             
            
            long  after = getusec();
        
             LOG( "Server", 3, "Parse ok. Action took " + toString((after-before)/1000) + "." + toString((after-before)%1000) + "ms" );
            
            //lock.unlock();
            
             LOG( "Server", 3, "Total socket waste time now: " + toString(wasted) );   
        } 
	if ( !socketCommands.empty() )  {
            shared_ptr< Command >  com = socketCommands.front();
            
            long  before = getusec();
            
             LOG( "Server", 3, "Parsing commands... ( " +toString(socketCommands.size()) + " left)" );

            if (com->parse() == PARSE_NOT_PROCESSED) {
                printf("Command not processed for some reason. Kept in the commands queue for further processing\n");
            }  else {
                // Only removes from queue if it has been really processed.
                socketCommands.pop();
            }             
            
            long  after = getusec();
        
             LOG( "Server", 3, "Parse ok. Action took "    + toString((after-before)/1000) + "." + toString((after-before)%1000)    + "ms" );
            
            //lock.unlock();
            
             LOG( "Server", 3, "Total socket waste time now: " + toString(wasted) );    
        }
    } 
    catch(...) {
         LOG( "Server", 0, "Exception in parsing commands. Commands may've been wasted." );
    }

    lock.unlock();
}

//-----------------------------------------------------------------------------------------------------------------
LocalServer::LocalServer()
                        : initialized( false )
{
    cs = static_cast<CST*>( CSproxy::Get() );
}

//-----------------------------------------------------------------------------------------------------------------
int LocalServer::AgentCount() const
{
    return agents.size();
}

//-----------------------------------------------------------------------------------------------------------------
shared_ptr< CSAgent >  LocalServer::GetAgent( int index ) const 
{
     LOG( "LocalServer",2, "Asking for agent at index " + toString(index));
    shared_ptr< CSAgent > result;
    if  (index >= 0 && index < AgentCount()) {
        int i = 0;
        for (set<shared_ptr<CSAgent> >::const_iterator itr = agents.begin(); itr != agents.end(); itr++) {
            if (i == index) {
                result =  *itr;
                break;
    }
            i++;
    }
    }
    return result;
}

//-----------------------------------------------------------------------------------------------------------------
shared_ptr< Demon >  LocalServer::GetDemon( int index ) const
{
    shared_ptr< Demon > result;
    if  (index >= 0 && index < DemonCount()) {
        int i = 0;
        for (set<shared_ptr<Demon> >::const_iterator itr = demons.begin(); itr != demons.end(); itr++) {
            if (i == index) {
                result =  *itr;
                break;
            }
            i++;
        }
    }
    return result;
    }
    
//-----------------------------------------------------------------------------------------------------------------
shared_ptr< Demon >  LocalServer::GetDemonByPort( int port ) const
    {
     LOG( "LocalServer",4, "Asking for demon at port " + toString(port));
     printf("Asking for demon at port %d\n", port);
    shared_ptr< Demon > result;
    for (set<shared_ptr<Demon> >::const_iterator itr = demons.begin(); itr != demons.end(); itr++) {
        shared_ptr<Demon> demon = *itr;
        printf("Found demon at port %d\n", demon->GetPort());
        if (demon->GetPort() == port) {
            result = demon;
        }
    }
    return result; 
}

//-----------------------------------------------------------------------------------------------------------------
void  LocalServer::DeleteDemon( std::string  name )
{
     for (set<shared_ptr<CSAgent> >::iterator itr = agents.begin(); itr != agents.end(); itr++) {
        shared_ptr<CSAgent> agent = *itr;
        if (!strcmp(name.c_str(),agent->GetName().c_str())) {
        iEngine*  engine  = FROM_CS( iEngine );
#if CRYSTAL
            engine->RemoveObject( agent->getCSBody() );
#else
            engine->RemoveObject( agent->getCSBody().get() );
#endif
            agents.erase(agent);
            break;
    }
     }
     for (set<shared_ptr<Demon> >::iterator itr = demons.begin(); itr != demons.end(); itr++) {
        shared_ptr<Demon> demon = *itr;
        if (!strcmp(name.c_str(),demon->GetName().c_str())) {
            demons.erase(demon);
            break;
        }
     }
}

//-----------------------------------------------------------------------------------------------------------------
void  LocalServer::DeleteDemonByPort( int port )
{
    for (set<shared_ptr<Demon> >::iterator itr = demons.begin(); itr != demons.end(); itr++) {
        shared_ptr<Demon> demon = *itr;
        if (port == demon->GetPort()) {
            demons.erase(demon);
            // Remove from agents set.
            for (set<shared_ptr<CSAgent> >::iterator itr2 = agents.begin(); itr2 != agents.end(); itr2++) {
                shared_ptr<CSAgent> agent = *itr2;
                if (!strcmp(demon->GetName().c_str(),agent->GetName().c_str())) {
                iEngine*  engine  = FROM_CS( iEngine );
#if CRYSTAL
                    engine->RemoveObject( agent->getCSBody() );
#else
                    engine->RemoveObject( agent->getCSBody().get() );
#endif
                    agents.erase(agent);
                    break;
                }
            }
            break;
        }
    }
}

//-----------------------------------------------------------------------------------------------------------------
int LocalServer::DemonCount() const
{
    return demons.size();
}

//-----------------------------------------------------------------------------------------------------------------
shared_ptr< Demon >  LocalServer::NewDemon( std::string  nick, float  x, float  y, float  z, ServerSocket*  socket)
{
    csVector3  startpos( x, y, z );    
    shared_ptr< Demon > demon( new Demon(startpos) );
    demon->SetSocket(socket);

    // Demon must have unique name/nick.
    std::string newname = nick;
    int i = 0;
    bool foundUniqueName;
    do {
        foundUniqueName = true;
        for (set<shared_ptr<Demon> >::const_iterator itr = demons.begin(); 
             itr != demons.end(); itr++) {
             shared_ptr< Demon> curDemon = *itr;
             if (!strcmp(curDemon->GetName().c_str(), newname.c_str())) {
                  foundUniqueName = false;
                  char itos[10];
                  sprintf(itos, "%i", i);
                  newname = nick+itos;
                  i++;
                  break;
             }    
         }
    } while(!foundUniqueName);
    demon->initialise( newname );
    demons.insert(demon);
    
     LOG( "LocalServer", 0, "Demon created." );            
    demon->UpdateView();

    return demon;    
}

//-----------------------------------------------------------------------------------------------------------------
shared_ptr< CSAgent >  LocalServer::NewAgent( shared_ptr<iMeshWrapper>  mesh_wrap, float  x, float  y, float  z, 
                                              ServerSocket*  socket)
{
    printf("LocalServer::NewAgent()\n");
    string name = mesh_wrap->QueryObject()->GetName();
    csVector3  startpos(x, y, z);
    shared_ptr< CSAgent > agent = shared_ptr< CSAgent > (new CSAgent(this, this, mesh_wrap, startpos ));
    
    agent->SetSocket( socket );
    agent->getRepresentation()->setListener( new PosListener( *(agent->getRepresentation()), *this ) );

    // Agent's body should be registered to a profile, too.
     LOG( "LocalServer", 0, "Agent created with name: " + name);
    if (IntConfig("CollisionDetection")) {
        printf("About to create colliders...\n");
        CSWorld::Get().CreateColliders();
    }
    agent->initialise( name );
    agents.insert(agent);
    demons.insert(agent);
    
    agent->UpdateView();
//    CSproxy::Get()->ForceRelight(); //is causing segfault with complex models like Child Robot
     LOG( "Server",1, "NewAgent ok." );

    return agent;
}
     
//-----------------------------------------------------------------------------------------------------------------
LocalServer::~LocalServer() {}
    
//-----------------------------------------------------------------------------------------------------------------
void LocalServer::Init()
{
    NewSocketThread();
    initialized = true;
}

//-----------------------------------------------------------------------------------------------------------------
Property*  LocalServer::findObjectFromRoot( std::string  s )
{    
    return NULL;
}

//-----------------------------------------------------------------------------------------------------------------
bool  LocalServer::UpdatePosition( double  dx, double  dy, double  dz, iMeshWrapper*  target)
{
    if ( !target  ||  !target->GetMovable() )
        return false;
    
    csVector3  newpos =    target->GetMovable()->GetPosition()    + csVector3( dx, dy, dz);    
      LOG( "LocalServer",2, string("Updating position of object '") + target->QueryObject()->GetName() + "' to (" + toString( newpos.x ) + "," + toString( newpos.y ) + "," + toString( newpos.z ) + ")");

    try {
        target->GetMovable()->SetPosition( newpos );
        target->GetMovable()->UpdateMove ();
    } catch(string s) {  LOG( "UpdatePosition",1,s); }
    catch(...) {  LOG( "UpdatePosition",1,"Exception!" ); }
    
    return true;
}

//-----------------------------------------------------------------------------------------------------------------
bool  LocalServer::ResetWorld( bool  hard )
{    
    if ( hard )
    {
         LOG( "Server", 3, "HARD Reset CSWorld..." );

        // Notify "GUI" to release the agent
        DisconnectFromWorld();    
        TheSocketManager.ClearAll();

        while (DemonCount() > 0 )  {
            shared_ptr<Demon> demon = *(demons.begin());
            string name = demon->GetName();
            LOG( "Server", 0, "Removing demon with name " + name );
            DeleteDemon(name);
        }        
         LOG( "Server", 3, "Reset CSWorld..." );
        CSWorld::ResetBridge();
         LOG( "Server", 3, "Reset SimWorld..." );
        SimWorld::ResetSingleton();

         LOG( "Server", 3, "Load new world..." );
        TheSimWorld.Load( StringConfig( "SimWorldFileName" ) );    

         LOG( "Server", 3, "Make new world..." );
        if ( !CSWorld::Get().MakeWorld() )  {
              LOG( "Server", 0, "Couldn't create CSWorld!" );
            return false;
        }    
         LOG( "Server", 3, "Take new view..." );

#if CRYSTAL
        cs->TakeView(CSWorld::Get().CreateView( csVector3( 0, 0, 0 ) ));    
#endif
         LOG( "Server", 3, "Register all objects..." );
        CSWorld::Get().RegisterAll(); //Must be done AFTER SimWorld is ok.
         LOG( "Server", 3, "World reset ok." );

        return cs->OnConnectToWorld();
    }
    else {
         LOG( "Server", 3, "SOFT Reset CSWorld..." );
        TheSocketManager.ClearAll();
        
         LOG( "Server", 3, "SKIP: Reset CSWorld..." );
        CSWorld::Get().ResetObjects();
        
        for (set<shared_ptr<CSAgent> >::iterator itr = agents.begin(); itr != agents.end(); itr++) {
            shared_ptr<CSAgent> agent = *itr;
            agent->resetProprioceptives();
        }

        // TODO: View not updated...            
    
         LOG( "Server", 3, "Re-register all objects..." );
        CSWorld::Get().RegisterAll(true);
                    
         LOG( "Server", 3, "World reset ok." );
        
        return true;
    }    
}

//-----------------------------------------------------------------------------------------------------------------
bool LocalServer::ConnectToWorld()
{
    try  {
         LOG( "LocalServer", 1, "Connecting..." );        
        Init();
         LOG( "LocalServer", 1, "Init ok." );
        
        return ResetWorld();                
    } catch( std::string s )  {
         LOG( "LocalServer", 0, s);
        return false;
    } catch(...) { 
         LOG( "LocalServer", 0, "Uknown exception." );
        return false;
    }
}

//-----------------------------------------------------------------------------------------------------------------
bool  LocalServer::DisconnectFromWorld()
{
    cs->GUIprovider->SetConnected(false);

    return true;
}

//-----------------------------------------------------------------------------------------------------------------    
void LocalServer::OnIdleEvent() //Could check collisions, etc.
{
    if ( !cs->GUIprovider->IsConnected() )
        return;

    DequeueCommands();

    if ( DemonCount() <= 0 )  {
         LOG( "Server", 4, "No demons found." );
        return;
    }

    try  {    
#if CRYSTAL
        csTicks  current_time,  elapsed_time;
          elapsed_time = cs->vc->GetElapsedTicks();
          current_time = cs->vc->GetCurrentTicks();
#else
          unsigned long elapsed_time = 1;
#endif

        set<shared_ptr<Demon> > markedToDeleteDemons;
        for (set< shared_ptr<Demon> >::iterator d = demons.begin(); d!= demons.end(); d++)  {
            shared_ptr<Demon> demon = *d;
                    
            // Check if demon's connection is ok
            ServerSocket* socket = demon->GetSocket();
            if (socket) {
                // Socket::Lost() method is under ENABLE_POOL definition, which is undefined at sockets-config.h
                //if (socket->Lost()) {
                //if (!socket->IsConnected() || !socket->Ready()) {
                if (!TheSocketManager.IsRegistered(socket)) {
                //if (!socket->IsConnected()) {
                    printf(">>>>>>>>> CONNECTION WAS LOST FOR DEMON %s <<<<<<<<<<<\n", demon->GetName().c_str());
                    markedToDeleteDemons.insert(demon);
                    break;
                }
            }
            
            std::map< std::string, shared_ptr<Sensor> >  senses    = demon->GetSenses();
            for (std::map< std::string, shared_ptr<Sensor> >::iterator  u = senses.begin(); u!= senses.end(); u++)    {
                  u->second->AddTics( elapsed_time );
                  u->second->AllowUpdate();
            }
#if CRYSTAL
            CSAgent*  bodied = dynamic_cast<CSAgent*>( demon.get() );
            if (bodied == NULL || !(IntConfig("EnableAnimation") && IntConfig("IndependentAnimationClock")) || 
                !CSWorld::Get().animated[bodied->getCSBody()->QueryObject()->GetName()]->Active()) {
#endif               
                demon->UnloadSensationQueue();
        }
        
        // Remove Demons whose connection was lost.
        for (set<shared_ptr<Demon> >::const_iterator itr = markedToDeleteDemons.begin(); 
             itr != markedToDeleteDemons.end(); itr++) {
            shared_ptr<Demon> demon = *itr;
            DeleteDemon(demon->GetName());
            // TODO: Remove demon's socket from SocketManager
            //TheSocketManager.delete(demon->GetSocket());
        }

         LOG( "SocketManager", 4, "Notify Flusher..." );
        TheSocketManager.FlushAll();
            
        lastflush = getusec();            
         LOG( "SocketManager", 4, "Notified Flusher." );        
        
    } catch(string s) {  LOG( "Server", 1, s.c_str()); }
    catch(...) {  LOG( "Server", 1, "Unknown exception." ); } 
            
     LOG( "Server",4,"Idle event ok" );        
}

//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
SimServer* SimServer::implementation = NULL;

//-----------------------------------------------------------------------------------------------------------------
SimServer*  SimServer::Get()
{
    if ( !implementation )
#if !LOCAL_SERVER
        implementation = new VOSServer();
#else
        implementation = new LocalServer();
#endif
    return implementation;    
}
    
//-----------------------------------------------------------------------------------------------------------------
const int  bounceMax = 50;

//-----------------------------------------------------------------------------------------------------------------
void  PosListener::BounceAll( set<string>&  non_bouncable )
{
    iMeshList*   meshes = FROM_CS( iEngine )->GetMeshes();
    set<string>  bounce_archive;
    set<string>  next_non_bouncable;

    for (set< string >::iterator  b = non_bouncable.begin(); b!= non_bouncable.end(); b++)  {
        iMeshWrapper*  mesh = meshes->FindByName( b->c_str() );
        string  oname;
        try {
            oname = ( mesh->QueryObject() ? mesh->QueryObject()->GetName() : "null" );
        } catch(...) { continue; }
        
        if ( !STLhas( TheSimWorld.GetMovableNames(), oname ) )    continue; //Walls are bounced with elsewhere.
        
        // LOG( "Bouncer", 1, ( "Bouncing " + oname).c_str());
        bounce_archive.clear();

        for (int bi = 0; bi < bounceMax; bi++)  {
            if ( !CSWorld::Get().CheckCollisions( mesh, non_bouncable, bounce_archive ) )
                bi = bounceMax;
        }
        
        for (set< string >::iterator  b = bounce_archive.begin(); b!= bounce_archive.end();    b++) {
            if ( !STLhas( non_bouncable, *b ) )
                next_non_bouncable.insert( *b );
        }
    }

    bool  new_moves = !next_non_bouncable.empty();
    
    for (set< string >::iterator  nb = next_non_bouncable.begin(); nb!= next_non_bouncable.end();    nb++)  {
        if ( !STLhas( non_bouncable, *nb ) )    {
            non_bouncable.insert( *nb );
        }
    }
    if ( !new_moves )
        return; //Everything that has moved has been handled now.

    BounceAll( non_bouncable );
}

//-----------------------------------------------------------------------------------------------------------------
void  PosListener::OnUpdate( const void*  obj )
{
    double  x, y, z;
    LocalObj3D * obj3d = (LocalObj3D*) static_cast<const LocalObj3D*>( obj );
    obj3d->getPosition( x, y, z);
    
    LOG( "PosListener", 2, "Setting position to (" + toString(x) + "," + toString(y) + "," + toString(z) + ")");
    
    shared_ptr<CSAgent> agent;
    shared_ptr<iMeshWrapper> agentMesh;
    for (set<shared_ptr<CSAgent> >::const_iterator itr = server.agents.begin(); itr != server.agents.end(); itr++) {
        shared_ptr<CSAgent> curAgent = *itr;
        if (curAgent->getRepresentation() == obj3d) {
            agent = curAgent;
            agentMesh = curAgent->getCSBody();
            break;
        }
    }
    if ( !agent.get() ) {
         LOG( "PosListener", 1, "Warning: Lacking agent. Update not done." );
        return;
    }

    agentMesh->GetMovable()->SetPosition( csVector3(x, y, z) );
    agentMesh->GetMovable()->UpdateMove ();

    LOG( "PosListener", 2, "UpdateMove() ok");
    
    /// Update the lifted object's position, if the agent's pos has changed.
    shared_ptr< meshData >  liftedo = agent->GetLifted();
    if ( liftedo.use_count() > 0 )  {
        liftedo->wrapper->GetMovable()->SetPosition( csVector3(x, y, z) + agent->GetLiftedObjectPosition());
        liftedo->wrapper->GetMovable()->UpdateMove();
    }

    if ( !IntConfig( "CollisionDetection" ) )  return;
    
    if ( currently_checking_collisions )       return;  //No regress wanted.
    
     LOG( "PosListener", 1, "Checking collisions...");
     
    currently_checking_collisions = true;
    
    set< string >     non_bouncable;
    set< string >     bounce_archive;    
    iEngine*  engine  = FROM_CS( iEngine );
    
    iMeshList*        meshes  = engine->GetMeshes();
    
    set< string >     wallset = CSWorld::Get().GetInertObjects();
    
#if 0
    // Do collision detection even if there is no walls
    if ( wallset.empty() )  {
         LOG( "Bouncer", 1, "Warning: Lacking 'walls' object. Collision detection not done." );
        return;
    }
#endif

    //    non_bouncable contains the things that no longer can be bounced.
    //    bounce_archive is just for book keeping.
        
    //     1. Agent pushes.

    non_bouncable.insert( AGENT_NAME );
    for (int bi = 0; bi < bounceMax; bi++) {
        if ( !CSWorld::Get().CheckCollisions( agentMesh.get(), non_bouncable, bounce_archive ) )
            bi = bounceMax;
    }

    float energycost = 0.0f;
    
    for (set< string >::iterator  b = bounce_archive.begin(); b!= bounce_archive.end();    b++) {    
        energycost += FloatConfig( "WalkStepLength" ) * TheSimWorld.GetWeight( *b ) * FloatConfig( "PricePerkgm" );
    }

    if ( bounce_archive.size() > 0 )  {
        char  t[200];
         sprintf(t, "Agent pushed %d objects with cost %.2f.\n", bounce_archive.size(), energycost);        
         LOG( "PosListener", 1, t);
        int   new_energy = (int)( _Int( agent->Get(WENERGY) ) - energycost );
        agent->Set( WENERGY, new_energy );
    }
        
    non_bouncable = bounce_archive;

    //    2. Other things push each other in queue.    
    BounceAll( non_bouncable );
    
    non_bouncable.clear ();    
    bounce_archive.clear();

    //    3. The walls bounce things back.
    for (set< string >::iterator  w = wallset.begin(); w!= wallset.end(); w++)  {
        string ws = *w;
        iMeshWrapper*  walls = meshes->FindByName( w->c_str() );

        for (int  b = 0; b < bounceMax; b++)  {
            if ( !CSWorld::Get().CheckCollisions( walls, non_bouncable, bounce_archive ) )
                b = bounceMax;
        }
        
        non_bouncable = bounce_archive;    
    }

    //    4. Other objects bounce things back, one at a time. Things bounced by the walls can't be re-bounced.
    BounceAll( non_bouncable );
    
    for(map< ServerSocket*, set<string> >::iterator  soci = CSWorld::Get().poslisteners.begin();
                                                soci!= CSWorld::Get().poslisteners.end(); soci++)  {
        for (set< string >::iterator  o = soci->second.begin(); o!= soci->second.end(); o++)  {            
            iMeshWrapper*  target = meshes->FindByName( o->c_str() );
            if ( target == NULL )  {
                 LOG( "Server", 2, "Listening to an unknown mesh object." );
                continue;
            }
            csVector3  v = target->GetMovable()->GetPosition();
                
            char  pos[30];
             sprintf( pos, "(%.2f,%.2f,%.2f)", v.x, v.y, v.z );
             printf( "*** server: calling hack LOG ***\n" );
            hacklog(XMLembed( "obj",
                                XMLembed("name", *o)
                              + XMLembed("pos", pos) ), NULL);
        }
    }
        
    currently_checking_collisions = false;

    // Sensors are updated by each agent's onAction method.
}
