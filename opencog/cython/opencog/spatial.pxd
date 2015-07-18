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
        void addSolidUnitBlock(cBlockVector,cHandle)
        void removeSolidUnitBlock(cHandle)
        cBlockVector getBlockLocation(cHandle)
        cHandle getBlock(cBlockVector)
