from cython.operator cimport dereference as deref, preincrement as inc
from opencog.atomspace cimport Handle



cdef class Block3D:
    cdef cBlock3D* cblock3d    

    def __init__(self,long addr):
        self.cblock3d = <cBlock3D*>PyLong_AsVoidPtr(addr)

    @classmethod
    def initNewBlock(self,int composedlevel,pos,material="",color="",candestroy=True):
        assert len(pos)==3
        cdef cBlockVector cpos = cBlockVector(pos[0],pos[1],pos[2])
        cdef string cmaterial = material.encode('UTF-8')
        cdef string ccolor = color.encode('UTF-8')
        cdef cBlock3D* cblock = new cBlock3D(composedlevel,cpos,
                                       cmaterial,ccolor,candestroy)
        newblock=self(PyLong_FromVoidPtr(cblock))
        return newblock
        

    def getLocation(self):
        cdef cBlockVector cpos=self.cblock3d.getPosition()
        return (cpos.x,cpos.y,cpos.z)
    
    def getMaterial(self):
        cdef cBlockMaterial cmat=self.cblock3d.getBlockMaterial()
        return cmat.materialType.decode('UTF-8')

    def getMaterial(self):
        cdef cBlockMaterial cmat=self.cblock3d.getBlockMaterial()
        return cmat.color.decode('UTF-8')

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
    def initNewMap(self,_mapName, _xMin, _yMin, _zMin,  _xDim, _yDim, _zDim, _floorHeight):
        cdef cOctree3DMapManager* cspmap = new cOctree3DMapManager(_mapName, _xMin, _yMin, _zMin,  _xDim, _yDim, _zDim, _floorHeight)
        newmap=self(PyLong_FromVoidPtr(cspmap))
        return newmap
    
    def getMapName(self):
        cdef string cname=self.coctree3dmap.getMapName()
        return cname.decode('UTF-8')

    def getObjectLocation(self,Handle objhandle):
        if self.coctree3dmap.containsObject(deref((<Handle>objhandle).h))==False:
            return None
        cdef cBlockVector cvec=self.coctree3dmap.getObjectLocation(deref((<Handle>objhandle).h))
        ret=(cvec.x,cvec.y,cvec.z)
        return ret
    
    def addSolidUnitBlock(self,pos,Handle handle,material,color):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        self.coctree3dmap.addSolidUnitBlock(cpos,deref((<Handle>handle).h),material,color)

    def getBlockAtLocation(self,pos):
        assert len(pos)==3
        cdef cBlock3D* cblock= self.coctree3dmap.getBlockAtLocation(pos[0],pos[1],pos[2])
        if <long>cblock == 0:
            return None
        ret=Block3D(<long>cblock)
        return ret
        
    def getUnitBlockHandleFromLocation(self,pos):
        assert len(pos)==3
        cdef cBlockVector cvec=cBlockVector(pos[0],pos[1],pos[2])
        cdef cHandle chandle=self.coctree3dmap.getUnitBlockHandleFromPosition(cvec)

        return Handle(chandle.value())
    
