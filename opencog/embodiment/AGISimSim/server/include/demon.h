/*
 * opencog/embodiment/AGISimSim/server/include/demon.h
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


#ifndef DEMON_H
#define DEMON_H

#include <queue>
#include "remoteobject.h"
#include "command.h"

class Sensor;
class ServerSocket;
class Property;
//class CustomSensation;
#include "sensation.h"

const char EYE_THETA[] = "eye.theta";
const char EYE_PHI[]   = "eye.phi";

//------------------------------------------------------------------------------------------------------------
/** @class Demon
 \brief A demon represents an abstract entity with sensors but no actuators.
 The demon moves around and reports its sensations via sockets.*/
//------------------------------------------------------------------------------------------------------------
class Demon : public RemoteObject
{
protected:
    ServerSocket*  socket;
    Obj3D*      body;
    int      port;
    string      nick;
    csVector3     startpos;

    /** The way of storing sense data for this observer. */
#if CRYSTAL
    csRef<iView>         view;
#endif
    std::queue<std::string>      sensationQueue;
    std::map<std::string, shared_ptr<Sensor> >  senses;

    /** Store this for the purposes of 'soft reset' */
    mutable boost::mutex                 lock_provider;
    mutable std::queue<CustomSensation>  customSensationQueue;

public:
    /** Use to construct an agent without a body */
    Demon (csVector3 _startpos);
    /** Use with agents to override default constructor functions */
    Demon (Obj3D* _body);
    virtual ~Demon();

    /** Insertion of a sensor into this demon
     \param __sname Sensor's exact case-insensitive name
     \param __sobject The sensor itself. */
    void InsertSense(string __sname, shared_ptr<Sensor> __sobject);

    /** Removal of a sensor from this demon \param __sname Sensor's exact case-insensitive name */
    void RemoveSense( string __sname);
    std::map<std::string, shared_ptr<Sensor> > GetSenses() const;

    virtual int initialise (std::string _nick);
    Obj3D*      getRepresentation() const;

    /** Send all accumulated sensory data to the socket */
    void UnloadSensationQueue();
    /** Force all sensors to check for updates immediately */
    void ForceSensorUpdate();

    /** Send the last data packet of each sensor. Because only changes
    are normally reported, this data must sometimes be queried explicitly. */
    void ResendSensorData (string sensorName = "all");

    /** CustomSensations are added in a manner different from other
    sensations, because there is no specific "source" for them. */
    void AddSensation (CustomSensation sensation) const;

#if CRYSTAL
    csRef<iView>  getView() const;
    void setView (csRef<iView> _view);
#endif
    void UpdateView();

    int    send   (std::string data, std::string destination);
    virtual ParseResult  action (std::vector<std::string>& parameters);
    virtual void onAction();

    virtual void ResetSensors();
    void    ResetVisionSensors();

    /** The socket must be set explicitly. The demon can exist without it. */
    void SetSocket (ServerSocket* _socket);
    ServerSocket*  GetSocket() const;

    /** Though only 1 non-agent demon is strictly supported, the demon is now identified by the socket port. */
    void SetPort (int _port);
    int  GetPort () const;

    string GetName() const;

    friend class Sensor;
    friend class SelfSensor;

};

#endif
