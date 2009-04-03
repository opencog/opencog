/*
 * opencog/embodiment/AGISimSim/server/include/space.h
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

//#pragma warning( disable : 4786 )

#define CRYSTAL 0

#ifndef SPACE_H
#define SPACE_H

#if CRYSTAL
#include <crystalspace.h>
#include "CSagent.h"
#else
#include "pseudospace.h"

#define CS_OPTS_H
#define NO_WX

#define csRef boost::shared_ptr

// class iMeshList;
class iMeshWrapper;
class iMovable;
class iEngine;
class iLightList;
class iCamera;
class iView;
class iImage;
class iGraphics2D;
class iMeshObjectFactory;
class csColliderWrapper;
class csCollisionPair;
class iSector;

#define SCREEN_W 640
#define SCREEN_H 480

class iLoader : public Singleton<iLoader>
{
public:
    int foo;
    friend class Singleton<iLoader>;
};

class iObject
{
    string name;
public:
    const char* GetName() const {
        return name.c_str();
    }
    void SetName(string _name) {
        name = _name;
    }
};

class iMovable
{
    csVector3 pos;
    csVector3 rot;
    iMeshWrapper* mesh;

public:
    iMovable(iMeshWrapper* mw) {
        mesh = mw;
    }
    iMovable(csVector3 _pos) : pos(_pos) {}
    csVector3 GetPosition() const {
        return pos;
    }
    csVector3 GetFullPosition() const {
        return pos;
    }
    void SetPosition(csVector3 _pos) {
        pos = _pos;
    }
    csVector3 GetRotation() const {
        return rot;
    }
    void SetRotation(csVector3 _rot) {
        rot = _rot;
    }
    void UpdateMove();
    iMeshWrapper* GetMesh() {
        return mesh;
    }
};

class iMeshWrapper
{
    csRef<iMovable> movable;
    csRef<iObject> obj;
public:
    string material;
    csVector3 dim;

    void GetWorldBoundingBox(csBox3& csdim) {
        csdim = csBox3(dim.x, dim.y, dim.z, dim.x, dim.y, dim.z);
    }

    iMeshWrapper() {
        movable = csRef<iMovable>(new iMovable(this)); obj = csRef<iObject>(new iObject);
    }
    iMovable* GetMovable() const {
        return movable.get();
    }
    iObject* QueryObject() const {
        return obj.get();
    }
    iMeshWrapper* GetMeshObject() {
        return this;
    }
    iMeshWrapper* GetObjectModel() {
        return this;
    }
    void GetObjectBoundingBox(csBox3& box) const {
        box = csBox3(-dim.x / 2, dim.x / 2, -dim.y / 2, dim.y / 2, -dim.z / 2, dim.z / 2);
    }
};

typedef iMeshWrapper iObjectModel;

class iMeshList
{
    std:: map<std::string, csRef<iMeshWrapper> > meshes;
    //std:: map<iMeshWrapper*, std::string> mptr2name;
public:
    iMeshWrapper* FindByName(string name) {
        return STLhas(meshes, name) ? meshes[name].get() : NULL;
    }
    int GetCount() const {
        return meshes.size();
    }
    iMeshWrapper* Get(int i) {
        int c = 0;
        map<string, csRef<iMeshWrapper> >::iterator it;
        for (it = meshes.begin(); it != meshes.end(); it++, c++)
            if (c == i)
                break;
        return it->second.get();
    }
    void Remove(iMeshWrapper* m) {
        //meshes.erase(mptr2name[m]);
        meshes.erase(m->QueryObject()->GetName());
        /*mptr2name.erase(m);*/
    }
    void Insert(csRef<iMeshWrapper> mw) {
        meshes[mw->QueryObject()->GetName()] = mw;
        //mptr2name[mw.get()] = mw->QueryObject()->GetName();
    }
    friend class iEngine;
};


typedef csVector3 csColor;

//------------------------------------------------------------------------------------------------------------
/** @class meshData
 \brief A helper structure to store objects that have been found in FOV. */
//------------------------------------------------------------------------------------------------------------
struct meshData {
    iMeshWrapper*  wrapper;
    int      index;
    float     distance;
    csVector3      intersect;

    meshData (iMeshWrapper*  _wrapper, int  _index, float _distance = 0.0f)
            : wrapper(_wrapper), index(_index), distance(_distance) {}
    meshData () : wrapper(0), index(0), distance(0.0f) {}

    friend class CollisionSystem;
};

struct xy {
    int x, y; xy(int _x, int _y) : x(_x), y(_y) {}
    bool operator<(const xy rhs) {
        return x < rhs.x || (x == rhs.x && y < rhs.y);
    }
};

struct lessxy : public binary_function<xy, xy, bool> {
    bool operator()(const xy& lhs, const xy& rhs) const {
        return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
    }
};


class iEngine : public Singleton<iEngine>
{
    csRef<iMeshList> meshes;
// std:: map<csVector3,csRef<iMeshWrapper> > pos2mesh;
    friend class Singleton<iEngine>;
    csVector3 amb;
    iLightList* ill;

    set<iMovable*>** blocks;
    map<iMovable*, set<xy, lessxy> > m2blocks;
    map<iMovable*, map<iMovable*, set<xy, lessxy> > > overlap;
    float dx, dy;

    int discrete_w, discrete_h;

    friend class PseudoObjectVisionSensor;

public:
    iEngine();
    iMeshList* GetMeshes () {
        return meshes.get();
    }
    void UpdateMove(iMovable* m);
    iMeshWrapper* FindMeshObject(string name) {
        return meshes->FindByName(name);
    }
    csRef<iMeshWrapper> CreateMeshWrapper(string material, string name, csVector3 pos, csVector3 dim) {
        csRef<iMeshWrapper> mesh(new iMeshWrapper());
        mesh->QueryObject()->SetName(name);
        mesh->material = material;
        mesh->GetMovable()->SetPosition(pos);
        mesh->dim = dim;

//  pos2mesh[pos] = mesh;
        meshes->Insert(mesh);

        UpdateMove(mesh->GetMovable());

        return mesh;
    }
    void DeleteAll() {}
    void SetLightingCacheMode(int i) {}
    void SetAmbientLight(csVector3 _amb) {
        amb = _amb;
    }
    void Prepare() {}
    void RemoveObject(iMeshWrapper* m) {
        meshes->Remove(m);
    }

    /// Ari's extras
    map< int, map<int, meshData> > GetVisibleObjects(float sx, float ex);
// map<int, map<int, iMeshWrapper*> > VisibleObjects(csVector3 orig, float sphi);
    map< int, map<int, meshData> > VisibleObjects(csVector3 orig, float sphi);
    void MeshDump();
    void graphDump() const;

    friend class CollisionSystem;
};

class csColliderWrapper
{
public:
    iMeshWrapper* mesh;
    csColliderWrapper(iMeshWrapper* _mesh) {
        mesh = _mesh;
    }
};

/*class iLightList
{
public:
};*/

/*class
{
public:
};
class
{
public:
};
class
{
public:
};*/

//------------------------------------------------------------------------------------------------------------
struct simplecoord {
    double x, y, z;
    simplecoord ()     : x(0.0), y(0.0), z(0.0) {}
    simplecoord (csVector3 v) : x(v.x), y(v.y), z(v.z) {}
    bool operator <(const simplecoord& rhs) const {
        return ( (x*x) + (y*y) + (z*z) ) < ( (rhs.x*rhs.x) + (rhs.y*rhs.y) + (rhs.z*rhs.z) );
    }
};

#undef FROM_CS
#define FROM_CS(c) (&(c::Instance()))

typedef unsigned int csEventID;

#include "CSproxy.h"

class CSAgent;

class CSpseudo : public CSproxy
{
private:
    CSstatus status;
    bool OnConnectToWorld();
public:
    static iGUIProvider*   GUIprovider;

    virtual ~CSpseudo() {}

    /** Initialize CS plugin system.  \param _GUIprovider The callback interface to the GUI. */
    bool InitcsPlugins( iGUIProvider* _GUIprovider ) {
        GUIprovider = _GUIprovider; return true;
    };

    /** Initialize CS after the plugin system is up. */
    CSstatus* OpenMainSystem() {
        return &status;
    }

    bool ConnectToWorld   (const char* worldURL);
    bool DisconnectFromWorld ();

    void OnGUIinit (int argc, char const * const argv[]) { }
    void OnGUIexit () { }

    /** Called by CS event handler to update the view. */
    void FinishFrame () { }
    bool SetupFrame  ();
    void PushFrame   ();
    void ForceRelight() { }
    void RunLoop     ();

    /** The GUI must give an instance of wxPanel to CS via this method.  \param panel The panel instance. */
    void Set2DPanel(void* panel) {}

    map< string, meshData >  GetMovableVisibleObjects( CSAgent*  csagent1 );

    friend class CSproxy;
    friend class LocalServer;
};

struct Screen : public Singleton<Screen> {
    int w, h;
    Screen() : w(SCREEN_W), h(SCREEN_H) {}

    friend class Singleton<Screen>;
};

#endif

#endif //SPACE_H
