/*
 * opencog/embodiment/AGISimSim/server/include/collisionsystem.h
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
    std::map<iMeshWrapper*, csColliderWrapper*>  colliders;
    std::map<ServerSocket*, std::set<std::string> >   poslisteners;
    CollisionSystem () {}
    void ClosestColliders (csCollisionPair *pairs, int N, csCollisionPair& closest);
public:
    virtual ~CollisionSystem();
    bool CreateColliders ();
    bool CheckCollisions (iMeshWrapper* inert, const std::set<std::string>& non_bouncable, std::set<std::string>& bounce_archive);
    bool InitCollider    (iMeshWrapper* mesh);

    void ListenObjectPosition(ServerSocket* listener, std::string objname);

    friend class PosListener;
};

#endif
