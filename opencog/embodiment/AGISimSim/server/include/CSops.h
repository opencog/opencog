/*
 * opencog/embodiment/AGISimSim/server/include/CSops.h
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


#ifndef CS_OPTS_H
#define CS_OPTS_H

#include <stdarg.h>
#include "simcommon.h"
#include "CSproxy.h"

struct iEngine;
struct iLoader;
struct iGraphics3D;
struct iKeyboardDriver;
struct iVirtualClock;
struct iObjectRegistry;
struct iEvent;
struct iSector;
struct iView;
struct iVosA3DL;
struct iAwsSource;
struct iPluginManager;
struct iVFS;
struct iCollideSystem;

class CSAgent;

//------------------------------------------------------------------------------------------------------------
struct simplecoord {
    double x, y, z;
    simplecoord ()     : x(0.0), y(0.0), z(0.0) {}
    simplecoord (csVector3 v) : x(v.x), y(v.y), z(v.z) {}
    bool operator <(const simplecoord& rhs) const {
        return ( (x*x) + (y*y) + (z*z) ) < ( (rhs.x*rhs.x) + (rhs.y*rhs.y) + (rhs.z*rhs.z) );
    }
};

//------------------------------------------------------------------------------------------------------------
/** @class meshData
 \brief A helper structure to store objects that have been found in FOV. */
//------------------------------------------------------------------------------------------------------------
struct meshData {
    iMeshWrapper*  wrapper;
    int      index;
    float     distance;
    csVector3      intersect;

    meshData (iMeshWrapper*  _wrapper, int  _index) : wrapper(_wrapper), index(_index) {}
    meshData () : wrapper(0), index(0), distance(0.0f) {}
};

//------------------------------------------------------------------------------------------------------------
/** @class CSbox
 \brief CS functionality provider.

 The class stores smart pointers to various CS plugins.
 It also stores a pointer to the CS object registry, from which
 the other plugins could be queried for.

 For easier access, you should use CS_FROM(i) macro, which retrieves
 the plugin with the interface i from the registry.
*/
//------------------------------------------------------------------------------------------------------------
class CSbox : public CSproxy
{
protected:
    static iObjectRegistry *object_reg;
    iSector* room;
    bool     GUI_driven_frames,  use_keyboard;

    csRef<iGraphics3D> g3d;
    csRef<iKeyboardDriver> kbd;
    csRef<iVirtualClock> vc;
    csRef<iGraphics2D> g2d;
    csRef<iPluginManager> pluginmanager;
    csRef<iLoader> loader;
    csRef<iVFS> filesys;
    csRef<iEngine> engine;

    /** The view shown on the screen */
    static csRef<iView> view;

    /** A view that can be chosen when no agents or demons are around */
    csRef<iView> neutral_view;

    // virtual bool InitEventHandler()=0;
    bool OnConnectToWorld();
public:
    virtual ~CSbox();

    static iGUIProvider*   GUIprovider;
    static bool      MyEventHandler (iEvent& ev);
    map<string, meshData>  GetMovableVisibleObjects(CSAgent* csagent1);

    /** Returns access to the CS object registry. */
    static iObjectRegistry* GetObjReg();

    /** Returns a view of an agent
     @param name The corresponding agent's name. */
    static csRef<iView> GetView(std::string name);

    /** Creates a new view for an agent
     @param name The corresponding agent's name. */
    static void ResetView(std::string name);

    /** Selects a new view for the screen
     @param _view The view to be acquired.*/
    bool   TakeView (csRef<iView> _view);
    static csRef<iVisibilityCuller> GetCuller();

    /* CSproxy implementation */
    void    PushFrame  ();
    CSstatus* OpenMainSystem();

    bool InitcsPlugins (iGUIProvider* _GUIprovider );
    void OnGUIinit    (int argc, char const * const argv[]);
    void FinishFrame   ();

    void Set2DPanel    (void* panel);
    void ForceRelight  ();
    void RunLoop    ();
    void OnGUIexit    ();

    friend class CSproxy;
    friend class LocalServer;
};

/** The quick way to get a CS plugin. */
#define FROM_CS(__queried_interface) csRef<__queried_interface>(CS_QUERY_REGISTRY(CSbox::GetObjReg(), __queried_interface))

#endif //CS_OPTS_H
