from libc.stdint cimport uint64_t
from libcpp cimport bool
from libcpp.string cimport string

from opencog.atomspace cimport cAtomSpace,cHandle, tv_ptr
from opencog.spatial cimport cOctree3DMapManager

cdef extern from "opencog/spacetime/Temporal.h":
    ctypedef uint64_t octime_t 

cdef extern from "opencog/spacetime/TimeServer.h" namespace "opencog":
    cdef cppclass cTimeServer "opencog::TimeServer":
        cTimeServer(cAtomSpace& a, cSpaceServer* ss)
        cHandle addTimeInfo(cHandle, octime_t, tv_ptr, string)

        

cdef extern from "opencog/spacetime/SpaceServer.h" namespace "opencog":

    ctypedef cOctree3DMapManager cSpaceMap

    cdef cppclass cSpaceServer "opencog::SpaceServer":
        cSpaceServer(cAtomSpace&)
        const cSpaceMap& getMap(cHandle)
        cHandle addOrGetSpaceMap(octime_t timestamp, string _mapName, int _xMin, int _yMin, int _zMin, int _xDim, int _yDim, int _zDim, int _floorHeight)
        bool addSpaceInfo(cHandle objectNode, cHandle spaceMapHandle, bool isSelfObject, octime_t timestamp,
                      int objX, int objY, int objZ,
                      int objLength, int objWidth, int objHeight,
                      double objYaw, string entityClass, string objectName, string material)
        void removeSpaceInfo(cHandle objectNode, cHandle spaceMapHandle, octime_t timestamp)
        void clear()
        void setTimeServer(cTimeServer*)


