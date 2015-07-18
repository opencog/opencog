from cython.operator cimport dereference as deref, preincrement as inc
from opencog.atomspace cimport Handle, AtomSpace

cdef class Octree3DMapManager:
    
    cdef cOctree3DMapManager* coctree3dmap 

    def __cinit__(self):
        pass

    def __init__(self, long addr):
        self.coctree3dmap = <cOctree3DMapManager*> PyLong_AsVoidPtr(addr)

    def __dealloc__(self):
        #spaceserver will handle this
        pass

    @classmethod
    def initNewMap(self,AtomSpace atomspace,mapName, resolution, floorHeight, agentHeight):
        cdef cOctree3DMapManager* cspmap = new cOctree3DMapManager(atomspace.atomspace,mapName, resolution,floorHeight, agentHeight)

        newmap=self(PyLong_FromVoidPtr(cspmap))
        return newmap
    
    def getMapName(self):
        cdef string cname=self.coctree3dmap.getMapName()
        return cname.decode('UTF-8')
    
    def addSolidUnitBlock(self,pos,Handle handle):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        self.coctree3dmap.addSolidUnitBlock(cpos,deref((<Handle>handle).h))

    def removeSolidUnitBlock(self,Handle handle):
        self.coctree3dmap.removeSolidUnitBlock(deref((<Handle>handle).h))

    def getBlock(self,pos):
        assert len(pos)==3
        cdef cBlockVector cvec=cBlockVector(pos[0],pos[1],pos[2])
        cdef cHandle cblock= self.coctree3dmap.getBlock(cvec)
        return Handle(cblock.value())
        
    def getBlockLocation(self,Handle handle):
        cdef cBlockVector cpos=self.coctree3dmap.getBlockLocation(deref((<Handle>handle).h))
        #TODO: in BlockVector it returns BlockVector::ZERO, which is (0,0,0)
        #if there's really one block in the zero point...
        if (cpos.x,cpos.y,cpos.z)==(0,0,0):
            return None
        return (cpos.x,cpos.y,cpos.z)
