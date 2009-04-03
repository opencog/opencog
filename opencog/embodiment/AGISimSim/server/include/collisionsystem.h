/*
 * opencog/embodiment/AGISimSim/server/include/collisionsystem.h
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

#ifndef COLLISIONSYSTEM_H
#define COLLISIONSYSTEM_H

class CSbox;
class GenericAgent;
class ServerSocket;

//------------------------------------------------------------------------------------------------------------
/** @class CollisionSystem
 \brief The collision detection and correction system. */
//------------------------------------------------------------------------------------------------------------
class CollisionSystem
{
protected:
    map<iMeshWrapper*, csColliderWrapper*>  colliders;
    map<ServerSocket*, set<string> >   poslisteners;
    CollisionSystem () {}
    void ClosestColliders (csCollisionPair *pairs, int N, csCollisionPair& closest);
public:
    virtual ~CollisionSystem();
    bool CreateColliders ();
    bool CheckCollisions (iMeshWrapper* inert, const set<string>& non_bouncable, set<string>& bounce_archive);
    bool InitCollider    (iMeshWrapper* mesh);

    void ListenObjectPosition(ServerSocket* listener, string objname);

    friend class PosListener;
};

#endif
