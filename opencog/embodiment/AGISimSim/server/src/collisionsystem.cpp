/*
 * opencog/embodiment/AGISimSim/server/src/collisionsystem.cpp
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

#include "simcommon.h"
#include "space.h"
#include "simworld.h"
#include "CSworld.h"
#include "collisionsystem.h"
#include "simserver.h"
#include <set>
#include "command.h"
#include "CSagent.h"

using namespace std;
using boost::shared_ptr;


#if CRYSTAL
#include "CSserver.h"

//--------------------------------------------------------------------------------------------------------------
void CollisionSystem::ClosestColliders(csCollisionPair *pairs, int N, csCollisionPair& closest)
{
		float closest_dist = 99999.0f;
		for (int i = 0; i < N;i++) //For each pair, compare all polygon pairs.	
		{
			float dist[9];
			dist[0] = ABS((pairs[i].a1 - pairs[i].a2).SquaredNorm());
			dist[1] = ABS((pairs[i].a1 - pairs[i].b2).SquaredNorm());
			dist[2] = ABS((pairs[i].a1 - pairs[i].c2).SquaredNorm());
			dist[3] = ABS((pairs[i].b1 - pairs[i].a2).SquaredNorm());
			dist[4] = ABS((pairs[i].b1 - pairs[i].b2).SquaredNorm());
			dist[5] = ABS((pairs[i].b1 - pairs[i].c2).SquaredNorm());
			dist[6] = ABS((pairs[i].c1 - pairs[i].a2).SquaredNorm());
			dist[7] = ABS((pairs[i].c1 - pairs[i].b2).SquaredNorm());
			dist[8] = ABS((pairs[i].c1 - pairs[i].c2).SquaredNorm());
			for (int j = 0; j<9; j++)
			{
				if (dist[i] < closest_dist)
				{
					closest = pairs[i];
					closest_dist = dist[i];
				}			
			}
		}
	}

//--------------------------------------------------------------------------------------------------------------
bool CollisionSystem::CheckCollisions (iMeshWrapper* inert, const set<string>& non_bouncable, 
									   set<string>&  bounce_archive)
{
	bool  crashes = false;

	try	{
		csRef<iEngine>  engine = FROM_CS (iEngine);
		iMeshList*      meshes = engine->GetMeshes();
			
		if (!meshes)  { LOG("Server", 1, "NO MESHES!");	}
		string       inertname (inert->QueryObject() ? inert->QueryObject()->GetName() : "<null>");
		set<string>  movablenames = TheSimWorld.GetMovableNames();
		
		for (int i = 0; i < meshes->GetCount(); i++) {
			iMeshWrapper*  target                = meshes->Get(i);
			string         targetname (target->QueryObject() ? target->QueryObject()->GetName() : "<null>");
			
			if ( targetname == inertname || movablenames.find(targetname ) == movablenames.end() )  continue;		
			 LOG("Server", 4, ("... Collider: " + targetname + " / " + inertname).c_str());

			csRef<iCollideSystem>  csys           = CSserver::GetColliderSystem();
			csys->ResetCollisionPairs();
			csColliderWrapper*     inertcolwrap   = csColliderWrapper::GetColliderWrapper (inert->QueryObject ());
			csColliderWrapper*     targetcolwrap  = csColliderWrapper::GetColliderWrapper(target->QueryObject ());
			iCollider*             inertcollider  = inertcolwrap->GetCollider();
			iCollider*             targetcollider = targetcolwrap->GetCollider();
            //printf("colliders %p / %p\n", targetcollider, inertcollider);

			if (1) {
				csReversibleTransform tr1 = inert->GetMovable()->GetFullTransform();
				csReversibleTransform tr2 = target->GetMovable()->GetFullTransform();
				

				bool  crash = csys->Collide(inertcollider, //colliders[inertname],
								&tr1,
								targetcollider, //colliders[targetname],
								&tr2);
                //if (crash) printf("CRASHED!!!\n");
				
				if ( crash && !STLhas (non_bouncable,targetname) ) {	
					crashes = true;
					
					csCollisionPair  cpair;
					ClosestColliders (csys->GetCollisionPairs(), csys->GetCollisionPairCount(),	cpair);
					
					LOG("Server", 1, (targetname + " was hit by "+inertname).c_str()); 
					
					csVector3        targetcenter;
					targetcenter = target->GetMovable()->GetPosition();
					csVector3  hitpos  ((cpair.a1.x + cpair.b1.x + cpair.c1.x)/3, (cpair.a1.y + cpair.b1.y + cpair.c1.y)/3, 
										(cpair.a1.z + cpair.b1.z + cpair.c1.z)/3);
					csVector3  ahitpos ((cpair.a2.x + cpair.b2.x + cpair.c2.x)/3, (cpair.a2.y + cpair.b2.y + cpair.c2.y)/3,	
										(cpair.a2.z + cpair.b2.z + cpair.c2.z)/3);
					csVector3  direction = targetcenter - hitpos;
					csVector3  hitdiff   = hitpos       - ahitpos;
					csPlane3   inert (cpair.a1, cpair.b1, cpair.c1);
					csVector3  normal    = inert.Normal();
					
					double     dx               = FloatConfig ("MinBounceOnCollision");				
					double     normal_amplitude = max (ABS(normal.x), max (ABS(normal.y),ABS(normal.z)));
					csVector3  fixer (dx * normal.x / normal_amplitude, 0.0, //dx*normal.y / normal_amplitude,
									dx * normal.z / normal_amplitude);
					csVector3  newpos = targetcenter + fixer;
					direction = fixer;

					target->GetMovable()->SetPosition (newpos);
					target->GetMovable()->UpdateMove  ();
					bounce_archive.insert(targetname);
					
					for (int i = 0; i < SimServer::Get()->AgentCount(); i++) {
						shared_ptr<CSAgent> agent = SimServer::Get()->GetAgent(i);
						if (nocase_equal(targetname.c_str(), agent->getCSBody()->QueryObject()->GetName())) {
							agent->getRepresentation()->setPosition (newpos.x, newpos.y, newpos.z);
							break;
						}
					}
				}
			}
		}
	} catch(...) { LOG("CollisionSystem",1, "Collision check failed."); return false; }

	return crashes;	
}

#else

void CollisionSystem::ClosestColliders(csCollisionPair *pairs, int N, csCollisionPair& closest)
{}

template<typename T,typename sortT>
bool intersect1(const set<T,sortT>& s1, const set<T,sortT>& t2, T& result);
  /*{
	for (set<T,sortT>::const_iterator i1 = s1.begin(); i1 != s1.end(); i1++)
	{
		if (t2.find(*i1) != t2.end())
		{
			result = *i1;
			return true;
		}
	}

	return false;
	}*/

bool CollisionSystem::CheckCollisions (iMeshWrapper* inert, const set<string>& non_bouncable, 
									   set<string>&  bounce_archive)
{
	bool  crashes = false;

	try	{
		iEngine*  engine = FROM_CS (iEngine);
		iMeshList*      meshes = engine->GetMeshes();
			
		if (!meshes)  { LOG("Server", 2, "NO MESHES!");	return false; }
		if (!inert)  { LOG("Server", 2, "NO INERT!");	return false; }

		string       inertname (inert->QueryObject() ? inert->QueryObject()->GetName() : "<null>");
		set<string>  movablenames = TheSimWorld.GetMovableNames();

		//set<xy, lessxy>& inert_blocks = iEngine::Instance().m2blocks[inert->GetMovable()];

		 LOG("Server", 3, "Collisions mesh-by-mesh...");
		
		for (int i = 0; i < meshes->GetCount(); i++)
		{
			 LOG("Server", 3, "Next mesh...");
			iMeshWrapper*  target                = meshes->Get(i);

			if (!target || !inert)
				continue;

			string         targetname (target->QueryObject() ? target->QueryObject()->GetName() : "<null>");
			
			if ( targetname == inertname || movablenames.find(targetname ) == movablenames.end() )  continue;		
			 LOG("Server", 3, ("... Collider: " + targetname + " / " + inertname).c_str());

/*			csRef<iCollideSystem>  csys           = CSserver::GetColliderSystem();
			csys->ResetCollisionPairs();
			csColliderWrapper*     inertcolwrap   = csColliderWrapper::GetColliderWrapper (inert->QueryObject ());
			csColliderWrapper*     targetcolwrap  = csColliderWrapper::GetColliderWrapper(target->QueryObject ());
			iCollider*             inertcollider  = inertcolwrap->GetCollider();
			iCollider*             targetcollider = targetcolwrap->GetCollider();*/
            //printf("colliders %p / %p\n", targetcollider, inertcollider);
	 
/*			 set<iMovable*> overlappers;
			 for (map<iMovable*, set<xy,lessxy> >::iterator o = iEngine::Instance().overlap.begin();
															o!= iEngine::Instance().overlap.end(); o++)
				if (intersect1(o->second, inert_blocks))
					overlappers.insert(o->first);				*/

			/*for (set<iMovable*>::iterator oo = overlappers.begin(); oo != overlappers.end(); oo++)
			{
				csVector3 targetcenter = (*oo)->GetPosition();
				
				float dist = (targetcenter - ).SquaredNorm
				
				center = o.center();
				if (dist < best_dist)
				{
					closest = o;
					best_dist = dist;
				}
				 
			 }*/

			//iMeshWrapper* cpair = intersect1(iEngine::Instance().m2blocks[inert], iEngine::Instance().m2blocks[target]);
//			bool  crash = (cpair!=NULL);

			//bool  crash = !overlappers.empty():
			 xy oxy(0,0);
			 bool  crash = false;

			 /// Should implement with find to avoid map's creating empty slots here:
			 crash = !iEngine::Instance().overlap[target->GetMovable()][inert->GetMovable()].empty();

/*			 if (STLhas(iEngine::Instance().overlap, target->GetMovable())
				 || STLhas(iEngine::Instance().overlap, inert->GetMovable()))
			 {
				LOG("Server", 1, "+++\n");

				crash = intersect1<xy,lessxy>(inert_blocks, iEngine::Instance().m2blocks[target->GetMovable()], oxy);
			 }*/

			/*csys->Collide(inertcollider, //colliders[inertname],
			&tr1,
			targetcollider, //colliders[targetname],
			&tr2);*/
			if (crash) LOG("Server", 3, "CRASHED!!!\n");
			
			if ( crash && !STLhas (non_bouncable,targetname) ) {	
				crashes = true;								
				
				LOG("Server", 2, (targetname + " was hit by "+inertname).c_str()); 
				
				csVector3  targetcenter = target->GetMovable()->GetPosition();
				csVector3  inertcenter = inert->GetMovable()->GetPosition();
//				csVector3  hitpos(xy.x, 0, xy.y);

//				csVector3  direction = targetcenter - hitpos;
				csVector3  normal = targetcenter - inertcenter;
				
				double     dx               = FloatConfig ("MinBounceOnCollision");	
				
				double     normal_amplitude = max (ABS(normal.x), ABS(normal.z));
				csVector3  fixer (dx * normal.x / normal_amplitude, 0.0, //dx*normal.y / normal_amplitude,
					dx * normal.z / normal_amplitude);
				csVector3  newpos = targetcenter + fixer;
				
				target->GetMovable()->SetPosition (newpos);
				target->GetMovable()->UpdateMove  ();
				bounce_archive.insert(targetname);
				
				for (int i = 0; i < SimServer::Get()->AgentCount(); i++) {
					shared_ptr<CSAgent> agent = SimServer::Get()->GetAgent(i);
					if (nocase_equal(targetname.c_str(), agent->getCSBody()->QueryObject()->GetName())) {
						agent->getRepresentation()->setPosition (newpos.x, newpos.y, newpos.z);
						break;
					}
				}
			}
		}
	} catch(...) { LOG("CollisionSystem",1, "Collision check failed."); return false; }
	 LOG("CollisionSystem",1, "CheckCollisions ok");
	return crashes;	
}

template<>
bool intersect1(const set<xy,lessxy>& s1, const set<xy,lessxy>& t2, xy& result)
{
	for (set<xy,lessxy>::const_iterator i1 = s1.begin(); i1 != s1.end(); i1++)
	{
		if (t2.find(*i1) != t2.end())
		{
			result = *i1;
			return true;
		}
	}

	return false;
}

#endif

//--------------------------------------------------------------------------------------------------------------
bool CollisionSystem::CreateColliders()
{
	try {
		iMeshList*  meshes    = FROM_CS(iEngine)->GetMeshes();
		int         realcount = 0;
		  
		for (int i = 0; i < meshes->GetCount();i++) {
			iMeshWrapper *mw = meshes->Get(i);
			if (!STLhas(colliders, mw))
			{
				InitCollider(mw);
				
				realcount++;
			}
		}

		 LOG("CollisionSystem", 1, toString(realcount) + " new colliders registered.");
	} catch(...) { return false; }

	return true;
}

//--------------------------------------------------------------------------------------------------------------
bool CollisionSystem::InitCollider (iMeshWrapper* mesh)
{
	 LOG("CollisionSystem",3,"Initializing new...");
	if (!mesh)  {
		 LOG("CollisionSystem",1,"ERROR 1");
		return false;
	}
#if CRYSTAL	
	if (!mesh->GetMeshObject())  {
		 LOG("CollisionSystem",1,"ERROR 2");
		return false;
	}
	if (!mesh->GetMeshObject()->GetObjectModel())  {
		 LOG("CollisionSystem",1,"ERROR 3");
		return false;
	}
	
    //printf("CollisionSystem::InitCollider(%s)\n", mesh->QueryObject()->GetName());
	iPolygonMesh*          polmesh = mesh->GetMeshObject()->GetObjectModel()->GetPolygonMeshColldet();
	csRef<iCollideSystem>  csys    = CSserver::GetColliderSystem();	
	 LOG("CollisionSystem",3, "Collision mesh retrieved.");
	
	if (polmesh) {
		colliders[mesh] = new csColliderWrapper(mesh->QueryObject(), csys, polmesh);	  
		 LOG("CollisionSystem",3, "Collider retrieved. Init ok.");	  
		return true;
	}
	else 
    {
         LOG("CollisionSystem",1, "Collider retrieval failed. polygonMesh not found!");    
        return false;
    }
#else
	colliders[mesh] = new csColliderWrapper(mesh);

	return true;
#endif
}

//--------------------------------------------------------------------------------------------------------------
CollisionSystem::~CollisionSystem()
{
	for (map<iMeshWrapper*, csColliderWrapper*>::iterator  i = colliders.begin();
		i != colliders.end(); i++)
	delete i->second;
}

//--------------------------------------------------------------------------------------------------------------
void CollisionSystem::ListenObjectPosition (ServerSocket* listener, string objname)
{
	poslisteners[listener].insert( objname );
}
