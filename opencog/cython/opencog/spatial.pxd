from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from opencog.atomspace cimport cHandle

cdef extern from "Python.h":
    # Tacky hack to pass atomspace pointer to AtomSpace ctor.
    cdef void* PyLong_AsVoidPtr(object)
    cdef object PyLong_FromVoidPtr(void *p)


cdef extern from "opencog/spatial/3DSpaceMap/Block3DMapUtil.h" namespace "opencog::spatial":
    cdef cppclass cBlockVector "opencog::spatial::BlockVector":
        cBlockVector()
        cBlockVector(int x,int y,int z)
        int x
        int y
        int z
    cdef cppclass cBlockMaterial "opencog::spatial::BlockMaterial":
        cBlockMaterial()
        cBlockMaterial(string,string)
        bool operator==()
        bool operator!=()
        string materialType
        string color

cdef extern from "opencog/spatial/3DSpaceMap/Block3D.h" namespace "opencog::spatial":
    cdef cppclass cBlock3D "opencog::spatial::Block3D":
        cBlock3D(int _composedLevel, cBlockVector& _position, string _materialType, string _color,bool _canDestroy)
        cBlockVector& getPosition()
        cBlockMaterial& getBlockMaterial()

cdef extern from "opencog/spatial/3DSpaceMap/Octree3DMapManager.h" namespace "opencog::spatial":
    cdef cppclass cOctree3DMapManager "opencog::spatial::Octree3DMapManager":
        cOctree3DMapManager(string _mapName, int _xMin, int _yMin, int _zMin, int _xDim, int _yDim, int _zDim, int _floorHeight)
        string getMapName()
        cBlockVector getObjectLocation(cHandle)
        bool containsObject(cHandle)
        void addSolidUnitBlock(cBlockVector,cHandle,string,string)
        cBlock3D* getBlockAtLocation(int,int,int)
        cHandle getUnitBlockHandleFromPosition(cBlockVector)
