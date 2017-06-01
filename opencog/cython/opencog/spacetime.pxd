from libc.stdint cimport uint64_t
from libcpp cimport bool
from libcpp.string cimport string

from opencog.atomspace cimport cAtomSpace, cHandle, tv_ptr
from opencog.spatial cimport cOpencogOcTree, cEntityRecorder

cdef extern from "opencog/spacetime/Temporal.h":
    ctypedef uint64_t octime_t

cdef extern from "opencog/spacetime/TimeServer.h" namespace "opencog":
    string DEFAULT_TIMEDOMAIN
    cdef cppclass cTimeServer "opencog::TimeServer":
        cTimeServer(cAtomSpace&, cSpaceServer*)
        cHandle addTimeInfo(cHandle, octime_t, string, tv_ptr)

cdef extern from "opencog/spacetime/SpaceServer.h" namespace "opencog":

    ctypedef cOpencogOcTree cSpaceMap

    cdef cppclass cSpaceServer "opencog::SpaceServer":
        cSpaceServer(cAtomSpace&)
        const cSpaceMap& getMap(cHandle)
        const cEntityRecorder& getEntityRecorder(cHandle)
        cHandle addOrGetSpaceMap(octime_t, string, double, string)
        bool addSpaceInfo(cHandle, cHandle, bool, bool, octime_t, double, double, double, string)
        void removeSpaceInfo(cHandle, cHandle, octime_t, string)
        void clear()
        void setTimeServer(cTimeServer*)
