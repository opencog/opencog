from cython.operator cimport dereference as deref

from opencog.atomspace cimport AtomSpace,Handle,TruthValue
from opencog.spatial import Octree3DMapManager as SpaceMap

#TO DEBUG:if we import opencog.spacetime before opencog.atomspace,
#once we get an undefined handle h1 in python, which UUID is -1
#then call h.is_undefined() method, it will return false!!
#After check the uuid of h1 and h1.h.UNDEFINED.value(),the latter becomes 0
#inside the is_undefined function!!
#It's strange, but I don't know how to debug this,
#to prevent this, you can choose import spacetime after the opencog.atomspace
#(but it will cause spacetime atom types not included in atomspace.types)
#or you can choose to check if it's undefined by h.value()==-1


cdef class TimeServer:
    cdef cTimeServer* ctimeserver

    def __cinit__(self,AtomSpace _a, SpaceServer _ss):
        self.ctimeserver= new cTimeServer(deref((<AtomSpace>_a).atomspace),_ss.cspaceserver)

    def __dealloc__(self):
        if self.ctimeserver:
            del self.ctimeserver

    def addTimeInfo(self,Handle h,timestamp, TruthValue tv=TruthValue(1.0,1.0e35), timeDomain="DefalutTimeDomain"):
        cdef string ctimeDomain=timeDomain.encode('UTF-8')
        cdef cHandle cattimelinkhandle = self.ctimeserver.addTimeInfo(deref((<Handle>h).h),timestamp, deref(tv._tvptr()),ctimeDomain)
        return Handle(cattimelinkhandle.value())


cdef class SpaceServer:

    cdef cSpaceServer *cspaceserver
    def __cinit__(self,AtomSpace _a):
        self.cspaceserver= new cSpaceServer(deref((<AtomSpace>_a).atomspace))
    def __dealloc__(self):
        if self.cspaceserver:
            self.cspaceserver.clear()
            del self.cspaceserver

    def getMap(self,Handle handle):
        mapinstance=SpaceMap(<long>(&(self.cspaceserver.getMap(deref((<Handle>handle).h)))))
        return mapinstance
    
    def addMap(self,timestamp, mapName,
               x,y,z,
               dimx,dimy,dimz, infloorHeight):
        cdef string cmapName=mapName.encode('UTF-8')
        cdef cHandle ch=self.cspaceserver.addOrGetSpaceMap(timestamp, cmapName,
                                                           x,y,z,
                                                           dimx,dimy,dimz,infloorHeight)
        pyhandle=Handle(ch.value())
        return pyhandle
    
    def addMapInfo(self,Handle objectNode,Handle spacemapNode,isSelfObject,timestamp,
                      objX, objY, objZ,
                      objLength, objWidth, objHeight,
                      objYaw,entityClass, objectName, material):
        cdef bool b=isSelfObject
        cdef bool ret=self.cspaceserver.addSpaceInfo(deref((<Handle>objectNode).h),deref((<Handle>spacemapNode).h),isSelfObject,timestamp,
                      objX, objY, objZ,
                      objLength, objWidth, objHeight,
                      objYaw,entityClass, objectName, material)
        return ret

    
    def removeMapInfo(self,Handle objectNode,Handle spacemapNode,timestamp):
        self.cspaceserver.removeSpaceInfo(deref((<Handle>objectNode).h),deref((<Handle>spacemapNode).h),timestamp)
    

    def clear(self):
        self.cspaceserver.clear()
    
    def setTimeServer(self,TimeServer timeserver):
        self.cspaceserver.setTimeServer(timeserver.ctimeserver)
    

    
