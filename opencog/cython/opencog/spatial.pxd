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


cdef extern from "opencog/spatial/3DSpaceMap/OpencogOcTree.h" namespace "opencog::spatial":

    cdef cppclass cOpencogOcTreeNode "opencog::spatial::OpencogOcTreeNode":
        float getLogOdds()

    cdef cppclass cOpencogOcTree "opencog::spatial::OpencogOcTree":
        cOpencogOcTree(string, double)

        #OpencogOcTree Inherited Interface

        float getOccupancyThresLog()
        void setOccupancyThres(float)

        cOpencogOcTreeNode* search(float, float, float)

        #OpencogOcTree Interface

        cHandle getBlock(cBlockVector, float)

        #Refactoring: Octree3DMapManager Interface

        string getMapName()
        int getAgentHeight()
        void setAgentHeight(float)
        int getTotalUnitBlockNum()

        void addSolidUnitBlock(cHandle, cBlockVector)
        void removeSolidUnitBlock(cHandle)
        void setUnitBlock(cHandle, cBlockVector, float)
        cBlockVector getBlockLocation(cHandle, float)


cdef extern from "opencog/spatial/3DSpaceMap/EntityRecorder.h" namespace "opencog::spatial":

    cdef cppclass cEntityRecorder "opencog::spatial::EntityRecorder":
        cEntityRecorder()
        cHandle getSelfAgentEntity()
        void addNoneBlockEntity(cHandle, cBlockVector, bool, bool, uint64_t)
        void removeNoneBlockEntity(cHandle)
        void updateNoneBlockEntityLocation(cHandle, cBlockVector, uint64_t)
        cBlockVector getLastAppearedLocation(cHandle)
        cHandle getEntity(cBlockVector)

cdef extern from "opencog/spatial/3DSpaceMap/SpaceMapUtil.h" namespace "opencog::spatial":
    bool checkStandableWithProb(cAtomSpace, cOpencogOcTree, cBlockVector, float)
    cBlockVector getNearFreePointAtDistance(cAtomSpace, cOpencogOcTree, cBlockVector, int, cBlockVector, bool)
