from libc.stdint cimport uint64_t
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from opencog.atomspace cimport cHandle, cAtomSpace

cdef extern from "Python.h":
    # Tacky hack to pass atomspace pointer to AtomSpace ctor.
    cdef void* PyLong_AsVoidPtr(object)
    cdef object PyLong_FromVoidPtr(void*)

cdef extern from "opencog/spatial/3DSpaceMap/Block3DMapUtil.h" namespace "opencog::spatial":
    cdef cppclass cBlockVector "opencog::spatial::BlockVector":
        cBlockVector()
        cBlockVector(double, double, double)
        double x
        double y
        double z


cdef extern from "opencog/spatial/3DSpaceMap/OctomapOcTree.h" namespace "opencog::spatial":

    cdef cppclass cOctomapOcTreeNode "opencog::spatial::OctomapOcTreeNode":
        float getLogOdds()

    cdef cppclass cOctomapOcTree "opencog::spatial::OctomapOcTree":
        cOctomapOcTree(cAtomSpace*, string, float, int, float)

        #OctomapOcTree Inherited Interface

        float getOccupancyThresLog()
        void setOccupancyThres(float)

        cOctomapOcTreeNode* search(float, float, float)

        #Refactoring: Octree3DMapManager Interface

        string getMapName()
        int getFloorHeight()
        int getAgentHeight()
        void setAgentHeight(float)
        int getTotalUnitBlockNum()
        cHandle getSelfAgentEntity()

        void addSolidUnitBlock(cHandle, cBlockVector)
        void removeSolidUnitBlock(cHandle)
        void setUnitBlock(cHandle, cBlockVector, float)
        cHandle getBlock(cBlockVector, float)
        cBlockVector getBlockLocation(cHandle, float)
        
        bool checkStandable(cBlockVector,float)

        void addNoneBlockEntity(cHandle, cBlockVector, bool, bool, uint64_t)
        void removeNoneBlockEntity(cHandle)
        void updateNoneBlockEntityLocation(cHandle, cBlockVector, uint64_t)
        cBlockVector getLastAppearedLocation(cHandle)
        cHandle getEntity(cBlockVector)

cdef extern from "opencog/spatial/3DSpaceMap/SpaceMapUtil.h" namespace "opencog::spatial":
    cBlockVector getNearFreePointAtDistance(cOctomapOcTree, cBlockVector, int, cBlockVector, bool)
