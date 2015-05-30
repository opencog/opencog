from libc.stdint cimport uint64_t
from libcpp cimport bool
from libcpp.string cimport string

from opencog.atomspace cimport AtomSpace,Handle,cAtomSpace,cHandle


#import octime_t
cdef extern from "opencog/spacetime/Temporal.h":
    ctypedef uint64_t octime_t #TODO:how to get the def of uint64_t

#import Octree3DMapManager
#TODO:it seems we should include it in another header..
cdef extern from "opencog/spatial/3DSpaceMap/Octree3DMapManager.h" namespace "opencog::spatial":
    cdef cppclass Octree3DMapManager:
        Octree3DMapManager(string _mapName, int _xMin, int _yMin, int _zMin, int _xDim, int _yDim, int _zDim, int _floorHeight)


cdef extern from "opencog/spacetime/SpaceServer.h" namespace "opencog":

    ctypedef Octree3DMapManager SpaceMap

    cdef cppclass SpaceServer "opencog::SpaceServer":
        SpaceServer(cAtomSpace&)
        const SpaceMap& getMap(cHandle)
        cHandle addOrGetSpaceMap(octime_t timestamp, string _mapName, int _xMin, int _yMin, int _zMin, int _xDim, int _yDim, int _zDim, int _floorHeight)
        bool addSpaceInfo(cHandle objectNode, cHandle spaceMapHandle, bool isSelfObject, octime_t timestamp,
                      int objX, int objY, int objZ,
                      int objLength, int objWidth, int objHeight,
                      double objYaw, bool isObstacle, string entityClass, string objectName, string material)
        void removeSpaceInfo(cHandle objectNode, cHandle spaceMapHandle, octime_t timestamp)
        void clear()
        


