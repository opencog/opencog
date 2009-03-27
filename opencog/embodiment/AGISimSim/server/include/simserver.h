/*
 * opencog/embodiment/AGISimSim/server/include/simserver.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
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

#ifndef SIMSERVER_H
#define SIMSERVER_H

#include <queue>
#include "space.h"

class CSbox;
class CSpseudo;
class CSAgent;
class Demon;
class Sensor;
class Command;
class ServerSocket;
class Property;
class iMeshWrapper;

#if CRYSTAL
 #include "CSserver.h"
 typedef CSbox		CST;
#else
 typedef CSpseudo	CST;
#endif

//------------------------------------------------------------------------------------------------------------
/** @class SimServer
	\brief The base class for server implementations. Though this is the
	bridge between possible implementations, only 1 [LocalServer] implementation
	currently exists.*/
//------------------------------------------------------------------------------------------------------------

 class SimServer{

private:
	static SimServer* implementation;

protected:
	/** Called from the socket system. Must be thread safe!	Use this if the commands are carried out immediately (no queue).	*/
	virtual void PerformCommand(shared_ptr<Command> sc)=0;

public:
	static SimServer* Get();

	virtual ~SimServer() {}

	/** Acquire the mutex that prevents the running of socket commands */
	virtual boost::mutex& AcquireLock()=0;

	/** Reset everything.
		\param hard If true, low-level reset is committed. Otherwise, objects
		are simply returned to their original positions. This is much faster,
		but some changes in the world may not be reversed.	*/
	virtual bool 	ResetWorld(bool hard) = 0;
	virtual void 	Init() 				  = 0;

	/** Possible future VOS support */
	virtual Property* 			findObjectFromRoot (std::string s) = 0;

	/** Get agent by index */
	virtual shared_ptr<CSAgent> GetAgent(int index) const=0;

    /** Get demon by index */
	virtual shared_ptr<Demon>   GetDemon(int index) const=0;

    /** Get demon by port */
    virtual shared_ptr<Demon>   GetDemonByPort(int port) const=0;

	/** Create a new agent. \param material The CS name of the texture (eg. 'wood').	*/
	virtual shared_ptr<CSAgent> NewAgent (csRef<iMeshWrapper> mesh_wrap, float x, float y, float z, ServerSocket* socket)=0;

	/** Create a new demon.	*/
	virtual shared_ptr<Demon>   NewDemon (std::string nick, float x,float y, float z, ServerSocket* socket = NULL)=0;
	/** Delete a demon or an agent. */
	virtual void 				DeleteDemon (std::string name) = 0;	
    /** Delete a demon or an agent. */
    virtual void                DeleteDemonByPort (int port) = 0; 
	
	/** Return the number of agents. */
	virtual int	 AgentCount () const = 0;
	/** Return the number of demons (including agents). */
	virtual int	 DemonCount () const = 0;	
	
	/** Postprocess an agent action. */
	virtual bool DisconnectFromWorld () = 0;
	virtual bool ConnectToWorld      () = 0;
	/** Called from CSbox's SetupFrame method */
	virtual void OnIdleEvent () = 0;
	/** Called from the socket system. Must be thread safe!	Use this if the commands will be performed from OnIdleEvent.	*/
	virtual void EnqueueCommand (shared_ptr<Command> sc) = 0;
    /** 
     * Enqueue new commands generated from a special command in a high priority queue, 
     * which should be processed before any other command in common queue. 
     */
    virtual void EnqueueHighPriorityCommand (shared_ptr<Command> sc) = 0;
	
	friend class LocalScriptWorld;
    friend class MapWorld;
};

class LocalServer;

//------------------------------------------------------------------------------------------------------------
/** @class PosListener
	\brief A position listener for world objects.

	This class works with CollisionSystem. Whenever the object moves,
	collisions are checked for. Currently this only works for LocalObj3D
	objects such as an agent body.*/
//------------------------------------------------------------------------------------------------------------
struct PosListener : public Listener {
	LocalObj3D  &obj;
	LocalServer &server;
	bool 		 currently_checking_collisions;
	
	PosListener (LocalObj3D &_obj, LocalServer &_server);
		
	void OnUpdate  (const void*);
	void BounceAll (set<string>& non_bouncable);
};

//------------------------------------------------------------------------------------------------------------
/** @class LocalServer
	\brief The server implementation.*/
//------------------------------------------------------------------------------------------------------------
class LocalServer : public SimServer  { //, public CollisionSystem
protected:
	CST* cs;
	set<shared_ptr<CSAgent> >  agents;
	set<shared_ptr<Demon> >    demons;

	boost::mutex socket_lock_provider;

	queue< shared_ptr<Command> > socketCommands;
    queue< shared_ptr<Command> > highPrioritySocketCommands;
	
	bool initialized;
	void Init();
	void InitView();

	/** Called from IdleEvent. Blocks the socket system.
		Must be thread safe! */
		
	void 		 DequeueCommands();
	virtual void PerformCommand(shared_ptr<Command> sc);

public:	
	LocalServer();
	virtual ~LocalServer();

	boost::mutex& AcquireLock();

	bool 				ResetWorld(bool hard = true);
	shared_ptr<CSAgent> GetAgent  (int index) const;
	shared_ptr<Demon>   GetDemon  (int index) const;
    shared_ptr<Demon>   GetDemonByPort  (int port) const;
	shared_ptr<CSAgent> NewAgent  (shared_ptr<iMeshWrapper> mesh_wrap, float x, float y, float z, ServerSocket* socket = NULL);
	shared_ptr<Demon> 	NewDemon  (std::string nick, float x,float y, float z, ServerSocket* socket = NULL);

	void 	DeleteDemon (std::string name);
    void    DeleteDemonByPort (int port);
	int 	AgentCount  () const;
	int 	DemonCount  () const;

	Property* findObjectFromRoot(std::string s);

	bool ConnectToWorld     ();
	bool DisconnectFromWorld();
	void OnIdleEvent		();
	void EnqueueCommand		(shared_ptr<Command> sc);
	void EnqueueHighPriorityCommand (shared_ptr<Command> sc);

    static bool  UpdatePosition( double  dx, double  dy, double  dz, iMeshWrapper*  target);

	friend class PosListener;
};



#endif
