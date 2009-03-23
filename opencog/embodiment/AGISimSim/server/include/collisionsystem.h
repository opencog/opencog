/***************************************************************************
 *  The collision detector.        
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

#ifndef COLLISIONSYSTEM_H
#define COLLISIONSYSTEM_H

class CSbox;
class GenericAgent;
class ServerSocket;

//------------------------------------------------------------------------------------------------------------
/** @class CollisionSystem
	\brief The collision detection and correction system. */
//------------------------------------------------------------------------------------------------------------
class CollisionSystem {
protected:
	map<iMeshWrapper*, csColliderWrapper*>  colliders;
	map<ServerSocket*, set<string> > 		poslisteners;
	CollisionSystem (){}
	void ClosestColliders (csCollisionPair *pairs, int N, csCollisionPair& closest);
public:
	virtual ~CollisionSystem();
	bool CreateColliders ();
	bool CheckCollisions (iMeshWrapper* inert, const set<string>& non_bouncable, set<string>& bounce_archive);
	bool InitCollider    (iMeshWrapper* mesh);

	void ListenObjectPosition(ServerSocket* listener,string objname);

	friend class PosListener;	
};

#endif
