/*
 * opencog/embodiment/AGISimSim/server/include/sensors.h
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


#ifndef SENSOR_H
#define SENSOR_H

#include <queue>
#include <map>

#include "sensation.h"
#include "simworld.h"
#include "remoteobject.h"
#include <boost/shared_array.hpp>

struct meshData;

//------------------------------------------------------------------------------------------------------------
/** Gives the brightness of the brightest light src at the point.
 Therefore brightness is NOT cumulative.*/
//------------------------------------------------------------------------------------------------------------
float GetBrightness(csVector3 point, iLightList* ll);

//------------------------------------------------------------------------------------------------------------
/** Check if there is an object at a specific point of the FOV.
 @param camera The camera (FOV) of the agent.
 @param x The x coordinate in the FOV.
 @param y The y coordinate in the FOV.
 @param poly The index number of the mesh. Currently not used, but
   you must store it.
 @param intersect Returns the 3D point of intersection. Useful for
   determining distance.
 @return The mesh object wrapper, if found.*/
//------------------------------------------------------------------------------------------------------------
iMeshWrapper* LocateObject (iCamera* camera, int x, int y, int* poly, csVector3& intersect);

#if CRYSTAL
//------------------------------------------------------------------------------------------------------------
/** Returns [x][y] = meshdata at (x,y) */
//------------------------------------------------------------------------------------------------------------
map< int, map<int, meshData> > GetVisibleObjects(csRef<iCamera> cam, const int sx, const int sy, const int ex, const int ey, const int netsize);

//------------------------------------------------------------------------------------------------------------
/** Returns [x][y] = meshdata at (x,y) */
//------------------------------------------------------------------------------------------------------------
map< int, map<int, meshData> > GetVisibleObjects (csRef<iCamera> cam, int w, int h, int netsize);

#endif

// TODO: Make sure this is thread safe!

extern std::queue<std::string>* sensationQueue;

//------------------------------------------------------------------------------------------------------------
/** @class FOVpos
 \brief A helper class for vision sensors.*/
//------------------------------------------------------------------------------------------------------------
struct FOVpos {
    FOVpos ( double _x, double _y, double _dist)  : x(_x), y(_y), dist(_dist) {}
    FOVpos () : x(0), y(0), dist(0) {}

    double x, y;
    double dist;

    std::string AsXML() const {
        return XMLembed ("fovpos",
                         XMLembed("x", toString(x)) + XMLembed("y", toString(y)) + XMLembed("dist", toString(dist)) );
    }
};

class Demon;

//------------------------------------------------------------------------------------------------------------
/** @class Sensor
 \brief Superclass of specific sensors (vision, sound, etc.).*/
//------------------------------------------------------------------------------------------------------------
class Sensor : public RemoteObject
{
protected:
    typedef unsigned int  uint;
    std::string     old_sense_data;
    std::string     sense_name;
    unsigned long      elapsed_tics;
    uint   w, h;
    bool   firstCall;
    Demon* demon;

    /** Subclasses may call this function whenever they have new sensation data available.*/
    void AddNewSensation(std::string new_sense_data);

public:
    Sensor  (Demon* _demon, std::string _sense_name, bool updateOnFirstCall = true);
    virtual ~Sensor ();

    void AddTics     (unsigned long ticks);
    void AllowUpdate ();
    void ForceUpdate ();
    void ClearCache  ();

    virtual void Update() = 0;

    static const nocase_string MODE;
    static const nocase_string POWERON;
};

//------------------------------------------------------------------------------------------------------------
/** @class ObjectVisionSensor sensors.h
 \brief Senses the objects by name and location in the demon's FOV.

Both VisionSensors should be given a NETSIZE paramater which determines the performance.
It defines the resolution of vision by the size of the smallest visible chunk.
Eg. netsize=10 means that only a single pixel of a 10x10 pixel area is checked.

ObjectVisionSensor produces sensations as
(pos, name, brightness)
where
pos = (x, y, distance)

Sample:
---
<visual>

<object>
  <fovpos>
 <x>60</x>
 <y>180</y>
 <dist>4000</dist>
  </fovpos>
  <name>walls_transp/north</name>
  <brightness>26</brightness>
</object>

<object>
...
</object>

</visual>
---
*/


class simplecoord;

//------------------------------------------------------------------------------------------------------------
class PseudoObjectVisionSensor : public Sensor
{
protected:
    int netsize;

public:
    PseudoObjectVisionSensor(Demon* _agent, bool updateOnFirstCall = true);
    virtual ~PseudoObjectVisionSensor();

    typedef std::map<std::string, std::set<simplecoord> > nmap;
    typedef std::map<std::string, float> mindistmap;
    typedef nocase_string OVUmode;

    void Update();
};

//------------------------------------------------------------------------------------------------------------
class MapInfoSensor : public Sensor
{

    std::map<std::string, std::string> removedObjects;
public:
    MapInfoSensor(Demon* _agent, bool updateOnFirstCall = true);
    virtual ~MapInfoSensor();
    void Update();
    void objectRemoved(const std::string& objName, const std::string& objType);
};


#if CRYSTAL
//------------------------------------------------------------------------------------------------------------
class ObjectVisionSensor : public Sensor
{
protected:
    csRef<iView>   view;
    csRef<iGraphics2D>  myG2D;

    int netsize;

public:
    /** Constructor.
     \param _mode ObjectVisionSensor has 2 modes:
     SINGLE_COMPONENT: Only return 1 "object" entry for each object in FOV. Gives the location and distance of the closest one.
     MULTI_COMPONENT : For each scanned pixel, return the object, if one is found. Thus for 1 object, many locations (and distances) may be found.
     \param _netsize : the pixel "resolution" of vision. Eg. size 10 implies that objects smaller than 10x10 pixels may go unobserved.*/
    ObjectVisionSensor (Demon* _demon, csRef<iGraphics2D> _myG2D,
                        csRef<iView> _view, //OVUmode _mode = "SINGLE_COMPONENT",
                        int _netsize = 10, bool updateOnFirstCall = true);
    virtual ~ObjectVisionSensor();

    typedef std::map<std::string, std::set<simplecoord> >   nmap;
    typedef std::map<std::string, float>      mindistmap;
    typedef nocase_string          OVUmode;
    static const OVUmode SINGLE_COMPONENT, MULTI_COMPONENT;

    void Update();
};

//------------------------------------------------------------------------------------------------------------
/** @class VisionSensor
 \brief Senses the pixels (or color blocks) in the demon's FOV.
 \param _netsize: the pixel "resolution" of vision. Eg. size 10 implies that objects smaller than 10x10 pixels may go unobserved.*/
//------------------------------------------------------------------------------------------------------------
class VisionSensor : public Sensor
{
protected:
    csRef<iImage>       bufferIm;
    csRef<iGraphics2D>  myG2D;
    csRef<iView>       view;
    int   netsize;
    uint  bufferW, bufferH;


public:
    VisionSensor (Demon* _demon, csRef<iGraphics2D> _myG2D, csRef<iView> _view,
                  int _netsize = 10, bool updateOnFirstCall = true);
    virtual ~VisionSensor();

    typedef nocase_string  VUmode;
    static const VUmode RAW, MEAN, SCALEFACTOR, REMOTE;

    void Update   ();
    void OnChange (csRef<iImage> im, uint w, uint h);

    bool Changed  (csRef<iImage> im1, uint w1, uint h1, csRef<iImage> im2, uint w2, uint h2);
    bool Save   (csRef<iImage> im, uint w, uint h, std::string path) const;
    bool DistanceSave    (std::string path, uint w, uint h) const;
    bool GetDistanceData (std::string& data, uint w, uint h) const;
    bool GetDistanceColorData (csRef<iImage> im, std::string& senseData, uint w, uint h) const;

    /** Return a string version of an image. The string is typically 100-1000kb long. */
    shared_array<unsigned char> AsciiImage(csRef<iImage> im, uint w, uint h) const;
};
#endif

//------------------------------------------------------------------------------------------------------------
/** @class AudioSensor
 \brief Auditory sensation handler. */
//------------------------------------------------------------------------------------------------------------
class AudioSensor : public Sensor
{
protected:

public:
    AudioSensor (Demon* _demon, bool updateOnFirstCall = true);
    virtual ~AudioSensor();

    void Update();
};

//------------------------------------------------------------------------------------------------------------
/** @class SmellSensor
 \brief Smell sensation handler. */
//------------------------------------------------------------------------------------------------------------
class SmellSensor : public Sensor
{
public:
    SmellSensor (Demon* _demon, bool updateOnFirstCall = true);
    virtual ~SmellSensor();

    void Update();
};

//------------------------------------------------------------------------------------------------------------
/** @class TasteSensor
 \brief Taste sensation handler.*/
//------------------------------------------------------------------------------------------------------------
class TasteSensor : public Sensor
{
protected:
    std::queue<Taste> tastes;

public:
    TasteSensor (Demon* _demon, bool updateOnFirstCall = true);
    virtual ~TasteSensor();
    /** Currently, the server calls this method of all taste sensors when
     a new taste was perceived. */
    void OnTaste (Taste taste);
    void Update  ();
};

class CSAgent;

//------------------------------------------------------------------------------------------------------------
/** @class TouchSensor
 \brief Touch sensation handler. (NOT IMPLEMENTED) */
//------------------------------------------------------------------------------------------------------------
class TouchSensor : public Sensor
{
protected:

public:
    TouchSensor(CSAgent* _agent, bool updateOnFirstCall = true);
    virtual ~TouchSensor();

    void Update();
};

//------------------------------------------------------------------------------------------------------------
/** @class SelfSensor
 \brief The demon's perception of its internal state (eg. position & energy).
 Also senses all the external "plugin" objects (eg. hand) connected to the demon.*/
//------------------------------------------------------------------------------------------------------------
class SelfSensor : public Sensor
{
public:
    SelfSensor(Demon* _demon);
    virtual ~SelfSensor();

    void Update();
};

#endif
