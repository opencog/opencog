/*
 * opencog/embodiment/AGISimSim/server/include/CSagent.h
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

#ifndef CSAGENT_H
#define CSAGENT_H

#include <iostream>
#include "demon.h"
#include "action_vocabulary.h"
#if CRYSTAL
#include "CSanimation.h"
#endif

class SimServer;
class Sensor;
class meshData;

//------------------------------------------------------------------------------------------------------------
/** @class CSAgent
 \brief The agent implementation on the server.*/
//------------------------------------------------------------------------------------------------------------
class CSAgent : public Demon
{
    float     rotx, roty;
    csVector3  startpos;
    double    agent_eye_phi, agent_eye_theta;
    Obj3D     *hand;
    csVector3  liftedObjectPos;
    shared_ptr<meshData>  liftedObject;

    csRef<iMeshWrapper> CS_body;

public:
    bool walkTowardsActionInProgress;
    bool nudgeToActionInProgress;
    bool dropBeforeNudgeTo;

    CSAgent (void* superobject, SimServer* _server, csRef<iMeshWrapper> _CS_body, csVector3 _startpos);
    ~CSAgent();

    virtual int     initialise (std::string nick);
    csRef<iMeshWrapper>   getCSBody  () const;

    void  GoTo(double x, double y, double z);
    bool  Lift(shared_ptr<meshData> object);
    bool  Drop();
    shared_ptr<meshData>  GetLifted() const;
    csVector3     GetLiftedObjectPosition() const;

    Obj3D* getRepresentation();
    Obj3D* getHand();

    /** Not really used. */
    virtual void disconnect();

    void onSensation(std::string m);

    /** Handle an action. Called typically from a socket command object. */
    virtual ParseResult action(std::vector<std::string>& parameters);
    virtual void onAction();

    /** Used by 'soft reset' */
    void resetProprioceptives();
    void ResetSensors();

    friend class LocalServer;
    friend class SelfSensor;
};

#endif
