from libc.stdint cimport uint64_t
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from opencog.atomspace cimport cHandle, cAtomSpace

cdef extern from "Python.h":
    # Tacky hack to pass atomspace pointer to AtomSpace ctor.
    cdef void* PyLong_AsVoidPtr(object)
    cdef object PyLong_FromVoidPtr(void *p)


cdef extern from "opencog/spatial/3DSpaceMap/Block3DMapUtil.h" namespace "opencog::spatial":
    cdef cppclass cBlockVector "opencog::spatial::BlockVector":
        cBlockVector()
        cBlockVector(double x,double y,double z)
        double x
        double y
        double z


cdef extern from "opencog/spatial/3DSpaceMap/Octree3DMapManager.h" namespace "opencog::spatial":
    cdef cppclass cOctree3DMapManager "opencog::spatial::Octree3DMapManager":
        cOctree3DMapManager(cAtomSpace*, string, unsigned, int, float)
        string getMapName()
        int getFloorHeight() 
        int getAgentHeight()
        void setAgentHeight(int)
        unsigned getTotalDepthOfOctree()
        int getTotalUnitBlockNum()
        float getLogOddsOccupiedThreshold()
        void setLogOddsOccupiedThreshold(float logOddsOccupancy)
        double getNextDistance()
        cBlockVector getKnownSpaceMinCoord()
        cBlockVector getKnownSpaceMaxCoord()
        cBlockVector getKnownSpaceDim()
        void addSolidUnitBlock(cHandle,cBlockVector)
        void removeSolidUnitBlock(cHandle)
        void setUnitBlock(cHandle,cBlockVector,float)
        bool checkIsSolid(cBlockVector)
        bool checkIsSolid(cBlockVector,float)
        bool checkIsStandable(cBlockVector)
        bool checkIsStandable(cBlockVector,float)
        cHandle getBlock(cBlockVector)
        cHandle getBlock(cBlockVector,float)
        cBlockVector getBlockLocation(cHandle)
        cBlockVector getBlockLocation(cHandle,float)
        float getBlockLogOddsOccupancy(cBlockVector)
        void addNoneBlockEntity(cHandle,cBlockVector,bool,bool,uint64_t)
        void removeNoneBlockEntity(cHandle)
        void updateNoneBlockEntityLocation(cHandle,cBlockVector,uint64_t)
        cBlockVector getLastAppearedLocation(cHandle)
