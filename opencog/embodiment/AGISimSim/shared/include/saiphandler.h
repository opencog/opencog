/*
 * opencog/embodiment/AGISimSim/shared/include/saiphandler.h
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

#ifndef _SAIPHANDLER_HH_
#define _SAIPHANDLER_HH_

#if 0

// TODO: Update to work without VOS.
class Agent;

//------------------------------------------------------------------------------------------------------------
template <class Agent>
class SAIPHandler
{
//protected:
    //int (*show)(std::string mimetype, std::string data );
    //int (*tell) (std::string data);
    //int (*ask) (std::string question);
    //int (*action) (std::string action, std::string parameters);
    //int (*mate) ();

public:
    SAIPHandler ();
    ~SAIPHandler();

    Agent* owner;
    void   setOwner (Agent* o);

    virtual void handleShow   (VOS::Message* m);
    virtual void handleTell   (VOS::Message* m);
    virtual void handleAsk    (VOS::Message* m);
    virtual void handleMate   (VOS::Message* m);
    virtual void handleAction (VOS::Message* m);

    //!!! Was ist das ???
    void setShow   (int (Agent::*s) (std::string mimetype, std::string data )) ;
    void setTell   (int (Agent::*t) (std::string data));
    void setAsk    (int (Agent::*a) (std::string question));
    void setAction (int (Agent::*a) (std::string action, std::string parameters));
    void setMate   (int (Agent::*m) ());

    //!!! Was ist das ???
    typedef int (Agent::* showCallback)(std::string mimetype, std::string data);
    showCallback show;
    typedef int (Agent::* tellCallback)(std::string data);
    tellCallback tell;
    typedef int (Agent::* askCallback)(std::string question);
    askCallback ask;
    typedef int (Agent::* actionCallback)(std::string action, std::string parameters);
    actionCallback action;
    typedef int (Agent::* mateCallback)();
    mateCallback mate;
};

#endif

#endif // _SAIPHANDLER_HH_
