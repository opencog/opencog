from cython.operator cimport dereference as deref

cimport spaceserver as defs

cdef class PySpaceMap:
    cdef const Octree3DMapManager* spmap

cdef class PySpaceServer:

    cdef defs.SpaceServer *spaceserver
    def __cinit__(self,AtomSpace _a):
        self.spaceserver= new defs.SpaceServer(deref((<AtomSpace>_a).atomspace))
    def __dealloc__(self):
        #TODO: not sure if the def of dtor is needed
        if self.spaceserver:
            del self.spaceserver
    def getMap(self,Handle handle):
        pymap=PySpaceMap()
        pymap.spmap=&self.spaceserver.getMap(deref((<Handle>handle).h))
        return pymap
    
#TODO:param is messy
    def addMap(self,long timestamp, mapName, xMin, yMin, zMin, xDim, yDim, zDim, infloorHeight):
        cdef string
        cdef cHandle ch=self.spaceserver.addOrGetSpaceMap(timestamp, mapName, xMin, yMin, zMin, xDim, yDim, zDim, infloorHeight)
        pyhandle=Handle()
        pyhandle.h=&ch
        return pyhandle
    
    def addMapInfo(self,Handle objectNode,Handle spacemapNode,isSelfObject,long timestamp,
                      objX, objY, objZ,
                      objLength, objWidth, objHeight,
                      objYaw, isObstacle, entityClass, objectName, material):
        cdef bool b=isSelfObject
        cdef bool ret=self.spaceserver.addSpaceInfo(deref((<Handle>objectNode).h),deref((<Handle>spacemapNode).h),isSelfObject,timestamp,
                      objX, objY, objZ,
                      objLength, objWidth, objHeight,
                      objYaw, isObstacle, entityClass, objectName, material)
        return True if ret==True else False

    
    def removeMapInfo(self,Handle objectNode,Handle spacemapNode,long timestamp):
        self.spaceserver.removeSpaceInfo(deref((<Handle>objectNode).h),deref((<Handle>spacemapNode).h),timestamp)
    

    def clear(self):
        self.spaceserver.clear()



    
